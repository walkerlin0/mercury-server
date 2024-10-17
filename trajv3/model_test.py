import json
import unittest

from trajv3.model import Ball


class TestModelMethods(unittest.TestCase):

    def test_object(self):
        b = Ball(location=(1,2,3), rotation_euler=(1,2,3))
        print(b.to_dict())