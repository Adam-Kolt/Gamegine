from setuptools import setup, find_packages

setup(
    name="frc_gamegine",
    version="0.1.0",
    packages=find_packages(include=["gamegine", "gamegine.*"]),
    install_requires=[
        "numpy>=1.21.0",
        "requests>=2.26.0",
    ],
    author="Adam Koltuniuk",
    author_email="adam@koltuni.uk",
    description=(
        "A FRC game representation, analysis, and simulation engine for "
        "performing post-kickoff strategy and design evaluation and optimization "
        "alongside ML Agent training."
    ),
    long_description=open("README.md", encoding="utf-8").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/yourusername/my_library",  # Update this URL
    classifiers=[
        "Programming Language :: Python :: 3",
    ],
    python_requires=">=3.10",
)
