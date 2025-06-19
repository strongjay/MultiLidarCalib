import unittest     # Python单元测试框架
import numpy as np  # 数值计算库

import ros2_numpy as rnp    # ROS 2与NumPy数据转换工具
from sensor_msgs.msg import PointCloud2, PointField, Image  # ROS 2传感器消息类型

# 自定义测试类和方法
class TestPointClouds(unittest.TestCase):

    '''生成随机点云数据'''
    def makeArray(self, npoints): # npoints: 点数
        points_arr = np.zeros((npoints,), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('r', np.uint8),
            ('g', np.uint8),
            ('b', np.uint8)])
        points_arr['x'] = np.random.random((npoints,))
        points_arr['y'] = np.random.random((npoints,))
        points_arr['z'] = np.random.random((npoints,))
        points_arr['r'] = np.floor(np.random.random((npoints,))*255)
        points_arr['g'] = 0
        points_arr['b'] = 255
        return points_arr

    '''测试数据类型转换'''
    def test_convert_dtype(self):
        fields = [
            PointField(name='x', offset=0, count=1, # 字段、在内存中的字节偏移量、数量、数据类型
                       datatype=PointField.FLOAT32),
            PointField(name='y', offset=4, count=1,  
                       datatype=PointField.FLOAT32)
        ]
        dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32)
        ])
        # NumPy类型→ROS消息转换
        conv_fields = rnp.msgify(PointField, dtype, plural=True)
        # 验证结果
        self.assertSequenceEqual(fields, conv_fields,
                                 'dtype->Pointfield Failed with simple values')

        conv_dtype = rnp.numpify(fields, point_step=8)
        self.assertSequenceEqual(dtype, conv_dtype,
                                 'dtype->Pointfield Failed with simple values')
    '''测试嵌套数据类型转换'''
    '''新增vectors字段（包含3个浮点数的数组）'''
    def test_convert_dtype_inner(self):
        fields = [
            PointField(name='x', offset=0, count=1,
                       datatype=PointField.FLOAT32),
            PointField(name='y', offset=4, count=1,
                       datatype=PointField.FLOAT32),
            PointField(name='vectors', offset=8, count=3,
                       datatype=PointField.FLOAT32)
        ]

        dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('vectors', np.float32, (3,))
        ])

        conv_fields = rnp.msgify(PointField, dtype, plural=True)
        self.assertSequenceEqual(fields, conv_fields,
                                 'dtype->Pointfield with inner dimensions')

        conv_dtype = rnp.numpify(fields, point_step=8)
        self.assertEqual(dtype, conv_dtype,
                         'Pointfield->dtype with inner dimensions')


    def test_roundtrip(self):

        points_arr = self.makeArray(100)
        cloud_msg = rnp.msgify(PointCloud2, points_arr)
        new_points_arr = rnp.numpify(cloud_msg)

        np.testing.assert_equal(points_arr, new_points_arr)

    def test_roundtrip_numpy(self):
        points_arr = self.makeArray(100)
        cloud_msg = rnp.msgify(PointCloud2, points_arr)
        new_points_arr = rnp.numpify(cloud_msg)

        np.testing.assert_equal(points_arr, new_points_arr)

    def test_roundtrip_zero_points(self):
        """Test to make sure zero point arrays don't raise memoryview.cast(*) errors"""
        points_arr = self.makeArray(0)
        cloud_msg = rnp.msgify(PointCloud2, points_arr)
        new_points_arr = rnp.numpify(cloud_msg)

        np.testing.assert_equal(points_arr, new_points_arr)

if __name__ == '__main__':
    unittest.main()