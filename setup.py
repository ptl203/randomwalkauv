from setuptools import setup

package_name = 'randomwalkauv'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paul',
    maintainer_email='paullahiff@gmail.com',
    description='AUV that does a random walk',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auv = randomwalkauv.auv:main',
            'env = randomwalkauv.env:main',
        ],
    },
)
