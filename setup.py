from setuptools import setup, find_packages

setup(
    name='frc_gamegine',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'numpy',
        'requests',
    ],
    author='Adam Koltuniuk',
    author_email='adam@koltuni.uk',
    description='A FRC game representation, analysis, and simulation engine for performing post-kickoff strategy and design evaluation and optimization alongside ML Agent training.',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/yourusername/my_library',
    classifiers=[
        'Programming Language :: Python :: 3',
    ],
    python_requires='>=3.11.9',
)
