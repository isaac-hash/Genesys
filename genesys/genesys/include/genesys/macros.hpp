#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp/qos.hpp>

#include <string>
#include <memory>
#include <vector>
#include <functional>
#include <type_traits>
#include <algorithm>
#include <chrono>

/* --------------------------------------------------------------------- *
 *  FNV-1a hash (runtime-safe; not constexpr because we use typeid().name)
 * --------------------------------------------------------------------- */
inline std::size_t fnv1a_hash(const char* str,
                              std::size_t hash = 14695981039346656037ULL)
{
    while (*str) {
        hash ^= static_cast<std::size_t>(*str++);
        hash *= 1099511628211ULL;
    }
    return hash;
}

template<typename T>
struct TypeHash {
    // typeid(T).name() is not constexpr -> compute at runtime but store as static
    static inline const std::size_t value = fnv1a_hash(typeid(T).name());
};

/* --------------------------------------------------------------------- *
 *  Metadata structs (required by the macros)                            *
 * --------------------------------------------------------------------- */
struct ParamInfo {
    std::string name;
    rclcpp::ParameterValue default_value;
    std::string attr_name;
};

using PublisherSetupFn =
    std::function<std::unique_ptr<struct IPublisher>(rclcpp::Node*, void*)>;

struct PubInfo {
    std::string topic;
    // ensure a valid default QoS so default ctor is well-formed
    rclcpp::QoS qos = rclcpp::QoS(10);
    std::size_t msg_type_hash;
    PublisherSetupFn setup_fn;
};

using SubscriptionBinderFn =
    std::function<std::unique_ptr<struct ISubscription>(rclcpp::Node*, void*)>;

struct SubInfo {
    std::string topic;
    rclcpp::QoS qos = rclcpp::QoS(10);
    std::size_t msg_type_hash;
    SubscriptionBinderFn setup_fn;
};

using TimerSetupFn =
    std::function<rclcpp::TimerBase::SharedPtr(rclcpp::Node*, void*)>;

struct TimerInfo {
    std::string name;
    std::chrono::milliseconds period;
    TimerSetupFn setup_fn;
};

/* --------------------------------------------------------------------- *
 *  Type-erasure interfaces                                             *
 * --------------------------------------------------------------------- */
struct IPublisher {
    virtual ~IPublisher() = default;
    virtual std::string get_topic_name() const = 0;
    virtual rclcpp::PublisherBase::SharedPtr get_base_pub() = 0;
};

template<typename MsgT>
struct PublisherImpl : IPublisher {
    explicit PublisherImpl(typename rclcpp::Publisher<MsgT>::SharedPtr p)
        : pub(std::move(p)) {}
    typename rclcpp::Publisher<MsgT>::SharedPtr pub;

    std::string get_topic_name() const override { return pub->get_topic_name(); }
    rclcpp::PublisherBase::SharedPtr get_base_pub() override { return pub; }
};

struct ISubscription {
    virtual ~ISubscription() = default;
    rclcpp::SubscriptionBase::SharedPtr sub;
};

template<typename MsgT>
struct SubscriptionImpl : ISubscription {
    explicit SubscriptionImpl(typename rclcpp::Subscription<MsgT>::SharedPtr s) { sub = std::move(s); }
};

/* --------------------------------------------------------------------- *
 *  Registries (one per concrete node class)                            *
 * --------------------------------------------------------------------- */
template<class NodeClass>
struct NodeRegistry {
    static inline std::vector<ParamInfo>   params;
    static inline std::vector<PubInfo>     pubs;
    static inline std::vector<SubInfo>     subs;
    static inline std::vector<TimerInfo>   timers;
};

/* --------------------------------------------------------------------- *
 *  Mixin – the heart of the framework                                  *
 * --------------------------------------------------------------------- */
template<typename NodeT>
class GenesysNodeMixin : public NodeT {
public:
    // Explicitly forward all constructor arguments
    template<typename... Args>
    explicit GenesysNodeMixin(Args&&... args)
        : NodeT(std::forward<Args>(args)...),
          param_handler_(this)   // <-- only pass node pointer; don't pass QoS here
    {
        _init_genesys();  // Auto-init on construction
    }

protected:
    std::vector<std::unique_ptr<IPublisher>>    _managed_pubs;
    std::vector<std::unique_ptr<ISubscription>> _managed_subs;
    std::vector<rclcpp::TimerBase::SharedPtr>   _managed_timers;

    rclcpp::ParameterEventHandler param_handler_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> param_cb_handle_;

    virtual void set_member(const std::string& attr_name,
                            const rclcpp::ParameterValue& value) = 0;

    virtual rcl_interfaces::msg::SetParametersResult
    on_parameter_event(const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& p : parameters) {
            const auto& reg = NodeRegistry<std::decay_t<decltype(*this)>>::params;
            auto it = std::find_if(reg.begin(), reg.end(),
                                   [&](const ParamInfo& info) { return info.name == p.get_name(); });
            if (it != reg.end()) {
                set_member(it->attr_name, p.get_parameter_value());
            }
        }
        return result;
    }

private:
    void _init_genesys()
    {
        declare_parameters();
        setup_publishers();
        setup_subscribers();
        setup_timers();

        param_cb_handle_ = param_handler_.add_parameter_callback(
            "",
            [this](const rclcpp::Parameter& p) {
                std::vector<rclcpp::Parameter> v{p};
                return this->on_parameter_event(v);
            },
            ""
        );
    }

    void declare_parameters()
    {
        const auto& reg = NodeRegistry<std::decay_t<decltype(*this)>>::params;
        for (const auto& info : reg) {
            this->declare_parameter(info.name, info.default_value);
            rclcpp::ParameterValue val = this->get_parameter(info.name).get_parameter_value();
            set_member(info.attr_name, val);
        }
    }

    void setup_publishers()
    {
        const auto& reg = NodeRegistry<std::decay_t<decltype(*this)>>::pubs;
        for (const auto& info : reg) {
            auto ptr = info.setup_fn(this, static_cast<void*>(this));
            _managed_pubs.push_back(std::move(ptr));
        }
    }

    void setup_subscribers()
    {
        const auto& reg = NodeRegistry<std::decay_t<decltype(*this)>>::subs;
        for (const auto& info : reg) {
            auto ptr = info.setup_fn(this, static_cast<void*>(this));
            _managed_subs.push_back(std::move(ptr));
        }
    }

    void setup_timers()
    {
        const auto& reg = NodeRegistry<std::decay_t<decltype(*this)>>::timers;
        for (const auto& info : reg) {
            auto timer = info.setup_fn(this, static_cast<void*>(this));
            _managed_timers.push_back(std::move(timer));
        }
    }
};

/* --------------------------------------------------------------------- *
 *  MACROS – user-facing API                                             *
 * --------------------------------------------------------------------- */

/* ---------- 1. Node base ------------------------------------------------ */
/* NOTE: GEN_NODE_END no longer auto-injects a set_member implementation.
   The user must provide their own GEN_SET_MEMBER_IMPL(ClassName) after the
   node declaration to avoid duplicate definitions. */
#define GEN_NODE_BEGIN(ClassName, node_name)                                  \
class ClassName final : public ::GenesysNodeMixin<rclcpp::Node> {             \
public:                                                                        \
    explicit ClassName(const rclcpp::NodeOptions& options = {})               \
        : ::GenesysNodeMixin<rclcpp::Node>(#node_name, options) {}            \
    virtual void set_member(const std::string&,                                \
                             const rclcpp::ParameterValue&) override;

#define GEN_NODE_END(ClassName)                                               \
};  /* end class */

/* ---------- 2. Lifecycle node ------------------------------------------- */
#define GEN_LIFECYCLE_NODE(ClassName, node_name)                              \
class ClassName final : public ::GenesysNodeMixin<rclcpp_lifecycle::LifecycleNode> { \
public:                                                                       \
    explicit ClassName(const rclcpp::NodeOptions& options = {})               \
        : ::GenesysNodeMixin<rclcpp_lifecycle::LifecycleNode>(#node_name, options) {} \
    virtual void set_member(const std::string&,                               \
                             const rclcpp::ParameterValue&) override;         \
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn \
    on_configure(const rclcpp_lifecycle::State&) override {                   \
        _init_genesys();                                                      \
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; \
    }                                                                         \
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn \
    on_cleanup(const rclcpp_lifecycle::State&) override {                     \
        _managed_pubs.clear();                                                \
        _managed_subs.clear();                                                \
        _managed_timers.clear();                                              \
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; \
    }                                                                         \
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn \
    on_activate(const rclcpp_lifecycle::State&) override {                    \
        for (auto& p : _managed_pubs) {                                       \
            auto base = p->get_base_pub();                                    \
            if (auto lc = std::dynamic_pointer_cast<                          \
                    rclcpp_lifecycle::LifecyclePublisher>(base)) {            \
                lc->on_activate();                                            \
            }                                                                 \
        }                                                                     \
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; \
    }                                                                         \
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn \
    on_deactivate(const rclcpp_lifecycle::State&) override {                  \
        for (auto& p : _managed_pubs) {                                       \
            auto base = p->get_base_pub();                                    \
            if (auto lc = std::dynamic_pointer_cast<                          \
                    rclcpp_lifecycle::LifecyclePublisher>(base)) {            \
                lc->on_deactivate();                                          \
            }                                                                 \
        }                                                                     \
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; \
    }                                                                         \
};  /* end lifecycle class */

/* ---------- 3. Parameter ------------------------------------------------ */
#define GEN_PARAM(ClassName, attr_name, param_name, default_value)            \
public:                                                                        \
    decltype(default_value) attr_name = default_value;                         \
private:                                                                       \
    struct _reg_param_##attr_name {                                            \
        _reg_param_##attr_name() {                                             \
            ParamInfo info{                                                    \
                #param_name,                                                   \
                rclcpp::ParameterValue(default_value),                         \
                #attr_name                                                     \
            };                                                                 \
            NodeRegistry<ClassName>::params.emplace_back(std::move(info));     \
        }                                                                      \
    };                                                                         \
    static inline _reg_param_##attr_name _reg_param_inst_##attr_name;

#define GEN_PARAM_CASE(attr_name, value_type)                                 \
    else if (attr_name == #attr_name) {                                       \
        this->attr_name = value.get<value_type>();                            \
    }

/* ---------- 4. Publisher (QoS depth) ------------------------------------ */
#define GEN_PUB(ClassName, attr_name, msg_type, qos_depth)                    \
private:                                                                      \
    PublisherImpl<msg_type>* _pub_impl_ptr_##attr_name = nullptr;             \
public:                                                                       \
    void attr_name(const msg_type& msg) {                                      \
        if (_pub_impl_ptr_##attr_name)                                         \
            _pub_impl_ptr_##attr_name->pub->publish(msg);                      \
    }                                                                          \
private:                                                                       \
    struct _reg_pub_##attr_name {                                              \
        _reg_pub_##attr_name() {                                               \
            PubInfo info;                                                      \
            info.topic = #attr_name;                                           \
            info.qos = rclcpp::QoS(qos_depth);                                 \
            info.msg_type_hash = ::TypeHash<msg_type>::value;                  \
            info.setup_fn = [info](rclcpp::Node* base, void* derived) mutable { \
                auto* node = static_cast<ClassName*>(derived);                 \
                auto pub = base->create_publisher<msg_type>(                   \
                    info.topic.c_str(), info.qos);                             \
                auto impl = std::make_unique<PublisherImpl<msg_type>>(pub);    \
                node->_pub_impl_ptr_##attr_name = impl.get();                  \
                return std::move(impl);                                        \
            };                                                                 \
            NodeRegistry<ClassName>::pubs.emplace_back(std::move(info));        \
        }                                                                      \
    };                                                                         \
    static inline _reg_pub_##attr_name _reg_pub_inst_##attr_name;

/* ---------- 5. Subscriber ----------------------------------------------- */
#define GEN_SUB(ClassName, attr_name, msg_type, qos_depth)                    \
public:                                                                       \
    void attr_name(std::shared_ptr<const msg_type> msg);                      \
private:                                                                      \
    struct _reg_sub_##attr_name {                                             \
        _reg_sub_##attr_name() {                                              \
            SubInfo info;                                                     \
            info.topic = #attr_name;                                          \
            info.qos = rclcpp::QoS(qos_depth);                                \
            info.msg_type_hash = ::TypeHash<msg_type>::value;                 \
            info.setup_fn = [info](rclcpp::Node* base, void* derived) mutable -> std::unique_ptr<ISubscription> { \
                auto* node = static_cast<ClassName*>(derived);                \
                auto cb = std::bind(&ClassName::attr_name, node, std::placeholders::_1); \
                auto sub = base->create_subscription<msg_type>(               \
                    info.topic.c_str(), info.qos, cb);                        \
                return std::make_unique<SubscriptionImpl<msg_type>>(std::move(sub)); \
            };                                                                \
            NodeRegistry<ClassName>::subs.emplace_back(std::move(info));      \
        }                                                                     \
    };                                                                        \
    static inline _reg_sub_##attr_name _reg_sub_inst_##attr_name;

/* ---------- 6. Timer ---------------------------------------------------- */
#define GEN_TIMER(ClassName, method_name, period_ms)                          \
public:                                                                       \
    void method_name();                                                       \
private:                                                                      \
    struct _reg_timer_##method_name {                                         \
        _reg_timer_##method_name() {                                          \
            TimerInfo info;                                                   \
            info.name = #method_name;                                         \
            info.period = std::chrono::milliseconds(period_ms);               \
            info.setup_fn = [info](rclcpp::Node* base, void* derived) {         \
                auto* node = static_cast<ClassName*>(derived);                 \
                return base->create_wall_timer(info.period, [node]() { node->method_name(); }); \
            };                                                                \
            NodeRegistry<ClassName>::timers.emplace_back(std::move(info));    \
        }                                                                     \
    };                                                                        \
    static inline _reg_timer_##method_name _reg_timer_inst_##method_name;

/* ---------- 7. Generate set_member implementation ---------------------- */
/* Removed automatic injection; user should define GEN_SET_MEMBER_IMPL(ClassName)
   once in their .cpp after the node declaration. */
#define GEN_SET_MEMBER_IMPL(ClassName)                                        \
    void ClassName::set_member(const std::string& attr_name,                  \
                               const rclcpp::ParameterValue& value)           \
    {                                                                         \
        if (false) {}                                                         \
        else {                                                                \
            RCLCPP_WARN(this->get_logger(),                                   \
                        "Unknown parameter: %s", attr_name.c_str());          \
        }                                                                     \
    }
