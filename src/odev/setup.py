from setuptools import find_packages, setup

package_name = 'odev'

def entry_point(name: str) -> str:
    return f"{name} = {package_name}.{name}:main"

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
    maintainer='ustad',
    maintainer_email='merenbasol@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            entry_point("odev_async"),
            entry_point("odev_sync"),
            entry_point("odev_b")
        ],
    },
)
