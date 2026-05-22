from setuptools import find_packages, setup

package_name = 'turtle_circle'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python', 'matplotlib'],
    zip_safe=True,
    maintainer='pietr',
    maintainer_email='ant.kapty@gmail.com',
    description='Turtle Draw: pipeline de visao computacional (do zero) + controle do turtlesim.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pipeline = turtle_circle.pipeline:main',
            'draw_node = turtle_circle.draw_node:main',
        ],
    },
)
