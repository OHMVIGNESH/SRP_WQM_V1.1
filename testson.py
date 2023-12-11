import json

dictionary = {'a':34, 'b':61, 'c':82}
jsonString = json.dumps(dictionary, indent=4)
print(jsonString)