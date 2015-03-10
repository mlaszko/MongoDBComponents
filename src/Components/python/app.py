import gridfs
import pymongo
import base64
import json
import bson
import sys
import httplib
from bottle import route, run, template, response,  get, post, request
from pymongo import MongoClient
from bson import json_util
from bson import BSON
from bson.objectid import ObjectId
from bson import Binary
import array
from bson import Binary, Code
from bson.json_util import dumps
connection = pymongo.MongoClient()
client = MongoClient('localhost', 27017)
db = client.images
containers = db.containers

@route('/static/objects')
def showObjectsInfo():
  
  data = list(containers.find({"NodeName":"Object"},{"_id":1,"NodeName":"Object","ObjectName":1, "description":1}))
  jsonList = json.dumps(data,default=json_util.default)
  jdata = json.loads(jsonList)
  table =  ''' 
  <style>
  table, th, td {
      border: 1px solid black;
      padding: 5px;
  }
  table {
      border-spacing: 15px;
  }
  </style>
  <table style="width:50%">
  <tr>
    <th>Object name</th>
    <th>Description</th>		
    <th>ID</th>
  </tr>'''
  for d in jdata:
      for key, value in d.iteritems():
	     if key=="ObjectName":
		  name=value
	     if key=="description":
		  description=value 
	     if key=="_id":
		  _id=value
      table = table + addRow(name, description, _id)   
  table = table + " </table>"
  return table + '''
	 <form action='/static/objects' method="post">
            ObjectName: <input name="ObjectName" type="text" />
	    <SELECT name="NodeName">
		<OPTGROUP label="NodeType">
		  <OPTION selected label="View" value="View">View</OPTION>
		  <OPTION label="Model" value="Model">Model</OPTION>
		</OPTGROUP>
	    </SELECT>
	    <input value="Get childs info" type="submit" />
	</form>
    '''

@route('/static/objects', method='POST')
def showViewsInfo():
  
  NodeName = request.forms.get('NodeName')
  ObjectName = request.forms.get('ObjectName')
  print NodeName, ObjectName
  client = MongoClient('localhost', 27017)
  db = client.images
  containers = db.containers
  data = list(containers.find({"NodeName":NodeName, "ObjectName":ObjectName},{"_id":1,"NodeName":1,"ObjectName":1, "ModelName":1,"ViewName":1,"description":1}))
  jsonList = json.dumps(data,default=json_util.default)
  jdata = json.loads(jsonList)
  if NodeName=="View":
   table =  '''   <style>
  table, th, td {
      border: 1px solid black;
      padding: 5px;
  }
  table {
      border-spacing: 15px;
  }
  </style>
  <table style="width:50%">
  <tr>
    <th>View Name</th>
    <th>Description</th>		
    <th>ID</th>
  </tr>'''
  
  if NodeName=="Model":  
    table =  '''   <style>
  table, th, td {
      border: 1px solid black;
      padding: 5px;
  }
  table {
      border-spacing: 15px;
  }
  </style>
  <table style="width:50%">
  <tr>
    <th>Model name</th>
    <th>Description</th>		
    <th>ID</th>
  </tr>'''
  
  typeName="ViewName"
  
  if NodeName=="View":
      typeName="ViewName"
  if NodeName=="Model":
      typeName="ModelName"
  
  for d in jdata:
      for key, value in d.iteritems():
	     if key==typeName:
		  name=value
	     if key=="description":
		  description=value 
	     if key=="_id":
		  _id=value
      table = table + addRow(name, description, _id)   
  table = table + " </table>"
  return table + '''
	 <form action='/static/img/gridfs/image' method="post">
	    <input type="hidden" name="ObjectName" value=%s>
	    <input type="hidden" name="ViewOrModel" value=%s>
            viewOrModelName: <input name="viewOrModelName" type="text" />
	    <SELECT name="NodeName">
		<OPTGROUP label="NodeName">
		  <OPTION selected label="StereoLR" value="StereoLR">StereoLR</OPTION>
		  <OPTION label="StereoPCXYZRGB" value="StereoPCXYZRGB">StereoPCXYZRGB</OPTION>
		  <OPTION label="StereoPCXYZSIFT" value="StereoPCXYZSIFT">StereoPCXYZSIFT</OPTION>
		  <OPTION label="StereoPCXYZSHOT" value="StereoPCXYZSHOT">StereoPCXYZSHOT</OPTION>
		  <OPTION label="ToFPCXYZSIFT" value="ToFPCXYZSIFT">ToFPCXYZSIFT</OPTION>
		  <OPTION label="ToFPCXYZRGB" value="ToFPCXYZRGB">ToFPCXYZRGB</OPTION>
		  <OPTION label="ToFPCXYZSHOT" value="ToFPCXYZSHOT">ToFPCXYZSHOT</OPTION>
		  <OPTION label="KinectPCXYZSHOT" value="KinectPCXYZSHOT">KinectPCXYZSHOT</OPTION>
		  <OPTION label="KinectPCXYZSIFT" value="KinectPCXYZSIFT">KinectPCXYZSIFT</OPTION>
		  <OPTION label="KinectPCXYZRGB" value="KinectPCXYZRGB">KinectPCXYZRGB</OPTION>
		  <OPTION label="StereoLR" value="StereoLR">StereoLR</OPTION>
		  <OPTION label="KinectRGBD" value="KinectRGBD">KinectRGBD</OPTION>
		  <OPTION label="ToFRGBD" value="ToFRGBD">ToFRGBD</OPTION>
		  <OPTION label="StereoRX" value="StereoRX">StereoRX</OPTION>
		  <OPTION label="KinectRX" value="KinectRX">KinectRX</OPTION>
		  <OPTION label="StereoRXM" value="StereoRXM">StereoRXM</OPTION>
		  <OPTION label="KinectRXM" value="KinectRXM">KinectRXM</OPTION>
		  <OPTION label="ToFRXM" value="ToFRXM">ToFRXM</OPTION>
		</OPTGROUP>
	    </SELECT>
	    <input value="Get childs info" type="submit" />
	</form>
    '''%(ObjectName,NodeName)

        
def addRow(name, description, _id):
  return '''  <tr>
    <td>%s</td>
    <td>%s</td>		
    <td>%s</td>
    </tr>    '''%(name, description, _id)

@route('/static/img/gridfs/getimage/<_id>/<contentType>/<extension>')
def get_img(_id,contentType,extension):
    dbname = 'containers'
    db = connection[dbname]
    fs = gridfs.GridFS(db)
    oid = ObjectId(str(_id))
    print oid
    #gridout = fs.get_last_version(filename=filename)
    gridout = fs.get(oid)
    response.content_type = contentType+"/"+extension
    return gridout

@route('/static/img/documents/getimage/<_id>/<contentType>/<extension>')
def retrieve_image(_id,contentType,extension):
    oid = ObjectId(str(_id))
    document = containers.find_one({"_id":oid})
    filename = document["filename"]
    data1 = json.loads(dumps(document))
    img = data1
    img1 = img[filename]
    img2 = json.loads(dumps(img1))
    print img2.get("$type",{})
    binary =  img2.get("$binary",{})
    decode = binary.decode()
    
    ##########
   # data = containers.find()
   # data1 = json.loads(dumps(data))
   # img = data1[29]
   # img1 = img['tempFile.png']
   # img2 = json.loads(dumps(img1))
   # print img2.get("$type",{})
   # binary =  img2.get("$binary",{})
   # decode = binary.decode()
    #################
    return '<img alt="sample" src="data:image/png;base64,{0}">'.format(decode)

@route('/static/img/gridfs/image', method='POST')
def view_images():
    # zabezpieczyc na wypadek nie znalezienia niczego
    viewOrModelName = request.forms.get('viewOrModelName')
    #silrx itp...
    NodeName = request.forms.get('NodeName')
    #View/Model
    ViewOrModel = request.forms.get('ViewOrModel')
    ObjectName = request.forms.get('ObjectName')
    
    print ViewOrModel
    print NodeName
    print ObjectName
    print viewOrModelName
    
    # get document
    if ViewOrModel=="View":
      data = list(containers.find({"NodeName":NodeName, "ObjectName":ObjectName, "ViewName":viewOrModelName },{"_id":1,"NodeName":1,"ObjectName":1,"ViewName":1,"description":1,"childOIDs":1}))
    if ViewOrModel=="Model":
      data = list(containers.find({"NodeName":NodeName, "ObjectName":ObjectName, "ModelName":viewOrModelName },{"_id":1,"NodeName":1,"ObjectName":1, "ModelName":1,"description":1,"childOIDs":1}))
   
    jsonList = json.dumps(data,default=json_util.default)
    jdata = json.loads(jsonList)
    print data
    # read childOIDs array
    for d in jdata:
      for key, value in d.iteritems():
	     if key=="childOIDs":
	       print value
	       childList = value
    foto2= ""
    #unpack array
    #iterate by all ids
    jsonList = json.dumps(childList,default=json_util.default)
    jdata = json.loads(jsonList)
    filename=""
    contentType=""
    foto=""
    place=""
    for d in jdata:
      for key, value in d.iteritems():
	if key=="childOID":
	  print value
	  oid = ObjectId(str(value))
	  #TODO zamienic na filename!!!
	  res = list(containers.find({"_id":oid},{"fileName":1, "size":1, "place":1, "extension":1, "contentType":1}))
	  print res
	  jsonList2 = json.dumps(res,default=json_util.default)
	  jdata2 = json.loads(jsonList2)
	  for d2 in jdata2:
	    for key2, value2 in d2.iteritems():
	      if key2=="filename":
		filename=value2
	      if key2=="fileName":
		filename=value2
	      if key2=="contentType":
		contentType=value2
	      if key2=="place":
		place=value2
		print place
	  if place=="grid":
	      path = "/static/img/gridfs/getimage/"+value+"/"+contentType;
	      foto = "<p> Focia %s:</p>  <p><img src=%s></p>" %(filename,path)
	  if place=="document":
	      document = containers.find_one({"_id":oid})
	      filename = document["filename"]
	      data1 = json.loads(dumps(document))
	      img = data1
	      img1 = img[filename]
	      img2 = json.loads(dumps(img1))
	      print img2.get("$type",{})
	      binary =  img2.get("$binary",{})
	      decode = binary.decode()
	      foto =  '<p> Focia %s:</p>  <p>  <img alt="sample" src="data:image/png;base64,{0}"></p>'.format(decode)%(filename)
	  foto2=foto2+foto   
    return foto2
    
run(host='localhost', port=8080)
