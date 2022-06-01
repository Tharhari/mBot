import requests

URL = "https://ims_api.supppee.workers.dev/api/coord"

r = requests.post(url = URL)

data = r.json()

print(data)

