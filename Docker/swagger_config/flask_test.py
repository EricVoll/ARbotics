import io
from flask import Flask, send_file

from flask_cors import CORS

# Cors

config = {
  'ORIGINS': [
    '*'
  ],

  'SECRET_KEY': '...'
}

# App

app = Flask(__name__)
CORS(app, resources={ r'/*': {'origins': config['ORIGINS']}}, supports_credentials=False)

file ='/home/AR-Manager_swagger_cfg.json'
@app.route("/cfg.json")
def logo():

    with open(file , 'rb') as bites:
        return send_file(
                     io.BytesIO(bites.read()),
                     attachment_filename='cfg.json',
                     mimetype='application/json'
               )
app.run(debug=True, host='0.0.0.0', port='5002') 

