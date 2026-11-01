from setuptools import find_packages, setup

package_name = 'pai_final_sorter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pai-student',
    maintainer_email='noreply@example.com',
    description='YOLOv8 の認識結果で CRANE+ V2 を仕分け動作させるパッケージ',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 「ros2 run pai_final_sorter sorter」で sorter_node.py の main() が呼ばれる
            'sorter = pai_final_sorter.sorter_node:main',
            # 認識だけを単体で試すためのコマンド
            'vision_test = pai_final_sorter.vision:main',
        ],
    },
)
