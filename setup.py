import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="simplecarsim-mopg",
    version="0.0.1",
    author="Max Opgenoord",
    author_email="11822896+mopg@users.noreply.github.com",
    description="Simple 2D car simulator",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/mopg/simplecarsim",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    package_dir={"": "src"},
    packages=setuptools.find_packages(where="src"),
    python_requires=">=3.6",
)