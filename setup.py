from setuptools import find_packages, setup

package_name = 'hsr_interface_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ht23a107',
    maintainer_email='ht23a107@oecu.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hsr_move_arm_node = hsr_interface_node.hsr_move_arm_node:main',
            'hsrb_pythoninterface_pose_goal_action_server = hsr_interface_node.hsrb_pythoninterface_pose_goal_action_server:main',
            'hsrb_pythoninterface_pose_goal_action_client = hsr_interface_node.hsrb_pythoninterface_pose_goal_action_client:main',
        ],
    },
)
