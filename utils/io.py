import pickle
import json
import os
import yaml

# -----------txt------------#
def read_txt(filename):
    with open(filename, 'r') as f:
        outinfo = f.read()
    return outinfo

def write_txt(filename, ininfo):
    with open(filename, 'w') as f:
        f.write(ininfo)

# -----------pickle----------#

def read_pickle(filename):
    try:
        with open(filename, 'rb') as f:
            return pickle.load(f)
    except:
        with open(filename, 'rb') as f:
            return pickle.load(f, encoding='latin1')

def write_pickle(filename, ininfo):
    with open(filename, 'wb') as f:
        pickle.dump(ininfo, f)

# -------------json-----------#
def read_json(filename):
    with open(filename, 'rb') as f:
        outinfo = json.loads(f.read())
    return outinfo

def write_json(filename, ininfo):
    with open(filename, 'wb') as f:
        json.dump(ininfo, f, indent=2, sort_keys=True)

# -----------folder operation------#
def folder_remove(foldername):
    if os.path.exists(foldername):
        print('Remove Folder: {}'.format(foldername))
        os.remove(foldername)

def isfile(filename):
    return os.path.exists(filename)

def make_folder(foldername):
    if not os.path.exists(foldername):
        print('Make folder: {}'.format(foldername))
        os.makedirs(foldername)

# ----------------yaml-----------#
def load_yaml(filename):
    with open(filename, 'r') as f:
        try:
            return yaml.safe_load(f)
        except yaml.YALMError as exc:
            raise exc






    
