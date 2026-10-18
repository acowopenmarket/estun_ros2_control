from setuptools import setup

setup(
    name='estun_segmented', version='0.1.0', packages=['estun_segmented'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/estun_segmented']),
        ('share/estun_segmented', ['package.xml']),
        ('share/estun_segmented/config', ['config/target.json'])],
    install_requires=['setuptools'], zip_safe=True,
    entry_points={'console_scripts': [
        'bridge = estun_segmented.bridge:main',
        'move = estun_segmented.runner:main']},
)
