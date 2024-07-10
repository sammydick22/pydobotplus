import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name='pydobotplus',
    packages=['pydobotplus'],
    version='0.1.0',
    description='Python library for Dobot Magician upgraded',
    long_description=long_description,
    long_description_content_type='text/markdown',
    author='sammydick22',
    author_email='imaterna@fit.vut.cz',
    url='https://github.com/sammydick22/pydobotplus',
    keywords=['dobot', 'magician', 'robotics', 'm1'],
    classifiers=[],
    install_requires=[
        'pyserial==3.4'
    ]
)
