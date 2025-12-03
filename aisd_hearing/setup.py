from setuptools import find_packages, setup

package_name = 'aisd_hearing'

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
    maintainer='root',
    maintainer_email='gundami0096@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aisd_hearing_node = aisd_hearing.aisd_hearing_node:main',
            'recording_publisher = aisd_hearing.recording_publisher:main',
            'words_publisher = aisd_hearing.words_publisher:main',
            'speak_client = aisd_hearing.speak_client:main'
        ],
    },
)
