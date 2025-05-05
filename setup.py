# setup.py
from setuptools import setup, find_packages

setup(
    name="dronenav",
    version="0.1",
    packages=find_packages(),
    install_requires=[
        "fastapi",
        "uvicorn[standard]",
        "numpy",
        "pydantic"
    ],
)