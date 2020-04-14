import requests 

url = " http://127.0.0.1:5000/"

r = requests.get(url=url)

print(r.text)
