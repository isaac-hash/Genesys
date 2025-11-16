import unittest
from genesys.decorators import component, publisher
from std_msgs.msg import String

class TestDecorators(unittest.TestCase):

    def test_component_metadata(self):
        @component('foo')
        class Foo:
            @publisher('out', String)
            def p(self):
                return String()
        
        self.assertTrue(getattr(Foo, '__genesys_is_component__'))
        self.assertEqual(getattr(Foo, '__genesys_component_name__'), 'foo')
        self.assertTrue(hasattr(Foo, 'get_node_factory'))

if __name__ == '__main__':
    unittest.main()
