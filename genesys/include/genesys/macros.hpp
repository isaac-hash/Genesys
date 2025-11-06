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

/* --------------------------------------------------------------------- *\n *  Compile-time FNV-1a hash (used only for sanity checks, not required) *\n * --------------------------------------------------------------------- */
constexpr std::size_t fnv1a_hash(const char* str,
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
    static constexpr std::size_t value = fnv1a_hash(typeid(T).name());
};

/* --------------------------------------------------------------------- *\n *  Metadata structs (required by the macros)                           *\n * --------------------------------------------------------------------- */
struct ParamInfo {
    std::string name;               // ROS parameter name
    rclcpp::ParameterValue default_value;
    std::string attr_name;          // C++ member name
};

using PublisherSetupFn =
    std::function<std::unique_ptr<struct IPublisher>(rclcpp::Node*, void*)>;

struct PubInfo {
    std::string topic;
    rclcpp::QoS qos;
    std::size_t msg_type_hash;
    PublisherSetupFn setup_fn;
};

using SubscriptionBinderFn =
    std::function<std::unique_ptr<struct ISubscription>(rclcpp::Node*, void*)>;

struct SubInfo {
    std::string topic;
    rclcpp::QoS qos;
    std::size_t msg_type_hash;
    SubscriptionBinderFn setup_fn;
};

using TimerSetupFn =
    std::function<rclcpp::TimerBase::SharedPtr(rclcpp::Node*, void*)>;

struct TimerInfo {
    std::string name;                     // just for debugging
    std::chrono::milliseconds period;
    TimerSetupFn setup_fn;
};

/* --------------------------------------------------------------------- *\n *  Type-erasure interfaces                                            *\n * --------------------------------------------------------------------- */
struct IPublisher {
    virtual ~IPublisher() = default;
    virtual std::string get_topic_name() const = 0;
    virtual rclcpp::PublisherBase::SharedPtr get_base_pub() = 0;
};

template<typename MsgT>
struct PublisherImpl : IPublisher {
    explicit PublisherImpl(rclcpp::Publisher<MsgT>::SharedPtr p)
        : pub(std::move(p)) {}
    rclcpp::Publisher<MsgT>::SharedPtr pub;

    std::string get_topic_name() const override { return pub->get_topic_name(); }
    rclcpp::PublisherBase::SharedPtr get_base_pub() override { return pub; }
};

struct ISubscription {
    virtual ~ISubscription() = default;
    rclcpp::SubscriptionBase::SharedPtr sub;
};

template<typename MsgT>
struct SubscriptionImpl : ISubscription {
    explicit SubscriptionImpl(rclcpp::Subscription<MsgT>::SharedPtr s) { sub = std::move(s); }
};

/* --------------------------------------------------------------------- *\n *  Registries (one per concrete node class)                           *\n * --------------------------------------------------------------------- */
template<class NodeClass>
struct NodeRegistry {
    static inline std::vector<ParamInfo>   params;
    static inline std::vector<PubInfo>     pubs;
    static inline std::vector<SubInfo>     subs;
    static inline std::vector<TimerInfo>   timers;
};

/* --------------------------------------------------------------------- *\n *  Mixin – the heart of the framework                                 *\n * --------------------------------------------------------------------- */
template<typename NodeT>
class GenesysNodeMixin : public NodeT {
public:
    using NodeT::NodeT;               // inherit ctors

protected:
    /* ---- managed ROS objects --------------------------------------- */
    std::vector<std::unique_ptr<IPublisher>>    _managed_pubs;
    std::vector<std::unique_ptr<ISubscription>> _managed_subs;
    std::vector<rclcpp::TimerBase::SharedPtr>   _managed_timers;

    /* ---- parameter handling ---------------------------------------- */
    rclcpp::ParameterEventHandler param_handler_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> param_cb_handle_;

    /* ---- virtual hook for the generated setter --------------------- */
    virtual void set_member(const std::string& attr_name,
                            const rclcpp::ParameterValue& value) = 0;

    /* ---- default parameter-change callback -------------------------- */
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
                set_member(it->attr_name, p.get_value<rclcpp::ParameterValue>());
            }
        }
        return result;
    }

    /* ---- internal init (called from the appropriate lifecycle hook) */
    void _init_genesys()
    {
        declare_parameters();
        setup_publishers();
        setup_subscribers();
        setup_timers();

        // single parameter callback for the whole node
        auto cb = [this](const std::vector<rclcpp::Parameter>& p) {
            return this->on_parameter_event(p);
        };
        param_handler_ = rclcpp::ParameterEventHandler(this);
        param_cb_handle_ = param_handler_.add_parameter_callback(cb);
    }

private:
    /* ---- parameter declaration & initial injection ------------------- */
    void declare_parameters()
    {
        const auto& reg = NodeRegistry<std::decay_t<decltype(*this)>>::params;
        for (const auto& info : reg) {
            this->declare_parameter(info.name, info.default_value);
            auto val = this->get_parameter(info.name).get_value<rclcpp::ParameterValue>();
            set_member(info.attr_name, val);
        }
    }

    /* ---- publishers ------------------------------------------------- */
    void setup_publishers()
    {
        const auto& reg = NodeRegistry<std::decay_t<decltype(*this)>>::pubs;
        for (const auto& info : reg) {
            auto ptr = info.setup_fn(this, static_cast<void*>(this));
            _managed_pubs.push_back(std::move(ptr));
        }
    }

    /* ---- subscribers ------------------------------------------------ */
    void setup_subscribers()
    {
        const auto& reg = NodeRegistry<std::decay_t<decltype(*this)>>::subs;
        for (const auto& info : reg) {
            auto ptr = info.setup_fn(this, static_cast<void*>(this));
            _managed_subs.push_back(std::move(ptr));
        }
    }

    /* ---- timers ----------------------------------------------------- */
    void setup_timers()
    {
        const auto& reg = NodeRegistry<std::decay_t<decltype(*this)>>::timers;
        for (const auto& info : reg) {
            auto timer = info.setup_fn(this, static_cast<void*>(this));
            _managed_timers.push_back(std::move(timer));
        }
    }
};

/* --------------------------------------------------------------------- *\n *  MACROS – user-facing API                                            *\n * --------------------------------------------------------------------- */

/* ---------- 1. Node base ------------------------------------------------ */
#define GEN_NODE(ClassName, node_name)                                        \
    class ClassName final : public ::GenesysNodeMixin<rclcpp::Node> {         \
    public:
        explicit ClassName(const rclcpp::NodeOptions& options =               \
                           rclcpp::NodeOptions())                             \
            : ::GenesysNodeMixin<rclcpp::Node>(node_name, options) {}         \
        virtual void set_member(const std::string&,                           \
                                 const rclcpp::ParameterValue&) override;     \
    protected:
        void post_construct() { _init_genesys(); }
    };

/* ---------- 2. Lifecycle node ------------------------------------------- */
#define GEN_LIFECYCLE_NODE(ClassName, node_name)                              \
    class ClassName final : public ::GenesysNodeMixin<rclcpp_lifecycle::LifecycleNode> { \
    public:
        explicit ClassName(const rclcpp::NodeOptions& options =               \
                           rclcpp::NodeOptions())                             \
            : ::GenesysNodeMixin<rclcpp_lifecycle::LifecycleNode>(node_name, options) {} \
        virtual void set_member(const std::string&,                           \
                                 const rclcpp::ParameterValue&) override;     \
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn \
        on_configure(const rclcpp_lifecycle::State&) override {
            _init_genesys();                                                  \
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; \
        }
                                                                          \
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn \
        on_cleanup(const rclcpp_lifecycle::State&) override {
            _managed_pubs.clear();                                            \
            _managed_subs.clear();                                            \
            _managed_timers.clear();                                          \
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; \
        }
                                                                          \
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn \
        on_activate(const rclcpp_lifecycle::State&) override {
            for (auto& p : _managed_pubs) {                                   \
                auto base = p->get_base_pub();                                \
                if (auto lc = std::dynamic_pointer_cast<                     \
                        rclcpp_lifecycle::LifecyclePublisherBase>(base)) {    \
                    lc->on_activate();                                        \
                }
            }
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; \
        }
                                                                          \
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn \
        on_deactivate(const rclcpp_lifecycle::State&) override {
            for (auto& p : _managed_pubs) {                                   \
                auto base = p->get_base_pub();                                \
                if (auto lc = std::dynamic_pointer_cast<                     \
                        rclcpp_lifecycle::LifecyclePublisherBase>(base)) {    \
                    lc->on_deactivate();                                      \
                }
            }
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; \
        }
    };

/* ---------- 3. Parameter ------------------------------------------------ */
#define GEN_PARAM(attr_name, param_name, default_value)                       \
    public:
        std::remove_reference_t<decltype(default_value)> attr_name = default_value; \
    private:
        struct _reg_param_##attr_name {
            _reg_param_##attr_name() {
                ::ParamInfo info{
                    #param_name,
                                  rclcpp::ParameterValue(default_value),
                                  #attr_name };
                ::NodeRegistry<ClassName>::params.emplace_back(std::move(info)); 
            }
        };
        static inline _reg_param_##attr_name _reg_param_inst_##attr_name;

/* ---------- 4. Publisher ------------------------------------------------ */
#define GEN_PUB(topic_name, msg_type, qos)                                    \
    private:
        ::PublisherImpl<msg_type>* _pub_impl_ptr_##topic_name = nullptr;      \
    public:
        void topic_name(const msg_type& msg) {
            if (_pub_impl_ptr_##topic_name) {
                _pub_impl_ptr_##topic_name->pub->publish(msg);
            } else {
                RCLCPP_ERROR(this->get_logger(),
                             "Publisher '%s' not initialized!", #topic_name);
            }
        }
    private:
        struct _reg_pub_##topic_name {
            _reg_pub_##topic_name() {
                ::PubInfo info;
                info.topic = #topic_name;
                info.qos   = qos;
                info.msg_type_hash = ::TypeHash<msg_type>::value;
                info.setup_fn = [](rclcpp::Node* base, void* derived) ->       \
                    std::unique_ptr<::IPublisher> {
                    auto* node = static_cast<ClassName*>(derived);
                    auto pub = base->create_publisher<msg_type>(#topic_name, qos);
                    auto impl = std::make_unique<::PublisherImpl<msg_type>>(std::move(pub)); 
                    node->_pub_impl_ptr_##topic_name = impl.get();            
                    return impl;
                };
                ::NodeRegistry<ClassName>::pubs.emplace_back(std::move(info)); 
            }
        };
        static inline _reg_pub_##topic_name _reg_pub_inst_##topic_name;

/* ---------- 5. Subscriber ----------------------------------------------- */
#define GEN_SUB(topic_name, msg_type, qos)                                    \
    public:
        void topic_name(std::shared_ptr<const msg_type> msg);
    private:
        struct _reg_sub_##topic_name {
            _reg_sub_##topic_name() {
                ::SubInfo info;
                info.topic = #topic_name;
                info.qos   = qos;
                info.msg_type_hash = ::TypeHash<msg_type>::value;
                info.setup_fn = [](rclcpp::Node* base, void* derived) ->       \
                    std::unique_ptr<::ISubscription> {
                    auto* node = static_cast<ClassName*>(derived);
                    auto cb = std::bind(&ClassName::topic_name, node,
                                        std::placeholders::_1);
                    auto sub = base->create_subscription<msg_type>(#topic_name, qos, cb);
                    return std::make_unique<::SubscriptionImpl<msg_type>>(std::move(sub)); 
                };
                ::NodeRegistry<ClassName>::subs.emplace_back(std::move(info)); 
            }
        };
        static inline _reg_sub_##topic_name _reg_sub_inst_##topic_name;

/* ---------- 6. Timer ---------------------------------------------------- */
#define GEN_TIMER(method_name, period_ms)                                     \
    public:
        void method_name();
    private:
        struct _reg_timer_##method_name {
            _reg_timer_##method_name() {
                ::TimerInfo info;
                info.name   = #method_name;
                info.period = std::chrono::milliseconds(period_ms);
                info.setup_fn = [](rclcpp::Node* base, void* derived) ->       \
                    rclcpp::TimerBase::SharedPtr {
                    auto* node = static_cast<ClassName*>(derived);
                    auto cb = std::bind(&ClassName::method_name, node);
                    return base->create_wall_timer(
                        std::chrono::milliseconds(period_ms), cb);
                };
                ::NodeRegistry<ClassName>::timers.emplace_back(std::move(info)); 
            }
        };
        static inline _reg_timer_##method_name _reg_timer_inst_##method_name;

/* ---------- 7. Helper for the generated set_member --------------------- */
#define GEN_SET_MEMBER_IMPL(ClassName)
    void ClassName::set_member(const std::string& attr_name,
                               const rclcpp::ParameterValue& value)
    {
        (void)value; /* silence unused warning when no params */
        if (false) {}

        #define GEN_PARAM_CASE(attr_name, Type)
                else if (attr_name == #attr_name) {
                    this->attr_name = value.get_value<Type>();
                }

        #define GEN_SET_MEMBER_END
                else {
                    RCLCPP_WARN(this->get_logger(),
                                "Unknown parameter attribute: %s", attr_name.c_str());
                }
    }
