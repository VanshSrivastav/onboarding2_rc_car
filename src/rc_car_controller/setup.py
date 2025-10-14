from setuptools import find_packages, setup
from setuptools.command.install import install as _install
import sys

class install(_install):
    def run(self):
        _install.run(self)
        # rewrite console script shebang to venv python
        import os, fileinput
        for script in os.listdir('install/lib/rc_car_controller'):
            if script in ['subscriber', 'joystick_node', 'joystick_receiver_node']:
                script_path = os.path.join('install/lib/rc_car_controller', script)
                for line in fileinput.input(script_path, inplace=True):
                    if line.startswith('#!'):
                        print(f'#!{sys.executable}')
                    else:
                        print(line, end='')

package_name = 'rc_car_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'python-can'],
    zip_safe=True,
    maintainer='vsrivast',
    maintainer_email='vsrivast@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # local/native joystick (uses pygame when running on machine that has the controller)
            'joystick_node = rc_car_controller.joystick_node:main',
            'joystick_receiver_node = rc_car_controller.joystick_receiver_node:main',
            'subscriber = rc_car_controller.subscriber:main',
        
        ],
    },
)
