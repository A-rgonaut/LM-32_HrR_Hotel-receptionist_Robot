from setuptools import find_packages, setup

package_name = 'progetto'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/progetto.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paolo',
    maintainer_email='paolo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arbitraggio = progetto.Arbitraggio:main',
            'braccialetti = progetto.BraccialettiManager:main',
            'controller = progetto.DiffRobotController:main',
            # 'pianifica = progetto.Pianifica:main',
            'server_llm = progetto.ServerLLM:main',
            'server_neo4j = progetto.ServerNeo4j:main',
            'server_onto = progetto.ServerOntologia:main'
        ],
    },
)
