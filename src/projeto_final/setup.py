from setuptools import find_packages, setup
import os

package_name = 'projeto_final'

# Função para coletar arquivos de diretórios específicos
def package_files(directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]
    data_files = []
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))
    return data_files

# Inclui os arquivos de configuração, lançamento, simulação e urdf
data_files = package_files(['config/', 'launch/', 'simulation/', 'urdf/'])
# Adiciona os arquivos padrão
data_files += [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name]),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Massiel',
    maintainer_email='massi00br@gmail.com',
    description='Projeto final de navegação utilizando ROS2.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'RobotControl = projeto_final.RobotControl:main',
            'AStar = projeto_final.AStar:main'
        ],
    },
)
