from setuptools import setup

package_name = 'gabiru_optimizer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyswarms'],
    zip_safe=True,
    maintainer='carlos',
    maintainer_email='carlos@usp.br',
    description='Nodo para publicar los params del PP optimizados para cada tramo',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pso_node = gabiru_optimizer.pso_node:main',
        ],
    },
)
