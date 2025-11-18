import unittest
import os
from genesys_cli.commands.templates import get_python_component_template

class TestCodegen(unittest.TestCase):

    def test_component_template(self):
        template = get_python_component_template('my_comp', 'MyComp')
        self.assertIn('@component("my_comp")', template)
        self.assertIn('class MyComp(Node):', template)
        self.assertIn('def get_node_factory():', template)
        self.assertIn('return NodeFactory(MyComp)', template)

if __name__ == '__main__':
    unittest.main()
