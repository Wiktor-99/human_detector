from setuptools import find_packages, setup

package_name = 'human_detector'

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
    maintainer='Wiktor Bajor',
    maintainer_email='wiktorbajor1@gmail.com',
    description='Human detector package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_detector = human_detector.human_detector:main'
        ],
    },
)