from setuptools import setup

package_name = 'psaf_utils'

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
    maintainer='PSAF',
    maintainer_email='ps-af@es.tu-darmstadt.de',
    description='Simple controller for the simulation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulation_controller = psaf_utils.psaf_sim_controller:main'
        ],
    },
)
