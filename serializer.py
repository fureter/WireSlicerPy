import os
import json

import jsonpickle


def encode(obj, output_dir, file_name):
    """
    Uses jsonpickle library to pickle the given object as a json file.

    :param obj: Object to serialize.
    :param output_dir: Directory to store the serialized json file to.
    :param file_name: Name of the file to create, the file extension will be appended.
    """
    enc = jsonpickle.encode(obj)
    if '.' not in file_name:
        str_format = '%s.json'
    else:
        str_format = '%s'

    with open(os.path.join(output_dir, str_format % file_name), 'wt') as fid:
        json.dump(enc, fid)


def decode(file_path):
    """
    Parses a json file defining a python object and returns the object.

    :param file_path: Filepath to the json object to de-serialize.
    :return: Object defined by the json file.
    """
    with open(file_path, 'r') as fid:
        data = json.load(fid)
        obj = jsonpickle.decode(data)
    return obj
