from setuptools import setup
from glob import glob
import os

package_name = 'boson_video_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seu_nome',
    maintainer_email='seu_email@example.com',
    description='Pacote para publicar vídeo da FLIR Boson em um tópico ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_publisher = boson_video_publisher.video_publisher:main'
        ],
    },
)

