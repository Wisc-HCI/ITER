
import json

def release(effort):
    return json.dumps({'name':'release','effort':effort})
