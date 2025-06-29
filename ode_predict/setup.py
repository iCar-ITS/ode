from setuptools import find_packages, setup

package_name = 'ode_predict'

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
    maintainer='zainir17',
    maintainer_email='zainirsyad@outlook.com',
    description='Predict node for ODE',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'predict_node = ode_predict.predict_node:main'
        ],
    },
)
