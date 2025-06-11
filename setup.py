from setuptools import setup, find_packages

setup(
    name='manipulator_mujoco',
    version='1.0.0',
    packages=find_packages(),
     include_package_data=True,  # <-- Add this line
    package_data={
        "manipulator_mujoco": [
            "assets/robots/**/*",
            "assets/world/**/*",# Add other asset patterns as needed
            "assets/mocap/**/*",# Add other asset patterns as needed
        ],
    },
    install_requires=[
        # List the packages and versions from requirements.txt
        line.strip() for line in open('requirements.txt')
    ],
)
