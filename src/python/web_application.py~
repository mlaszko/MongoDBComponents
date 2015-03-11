import gridfs
import pymongo
import base64
import json
import bson
import sys
import httplib
import yaml
import os	
import time
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
#client = MongoClient('localhost', 27017)
client = MongoClient('/home/lzmuda/mongoSock/mongodb-27017.sock')
db = client.images
collection = db.containers
FileList = []
viewsSet = []

viewsSetNames = []
fileTypesList = ["ImageRgb", "FileCameraInfo", "ImageXyz",  "ImageDepth", "ImageIntensity", "ImageMask", "StereoLeft", "StereoRight", "StereoLeftTextured", "StereoRightTextured",
"PCXyz", "PCXyzRgb", "PCXyzSift", "PCXyzRgbSift", "PCXyzShot", "PCXyzRgbNormal", "Stereo", "StereoTextured"]
    
@route('/static/objects/views')
def showViewsInfo():
  print  "showViewsInfo"
  print "delete List"
  del FileList[:]
  del viewsSet[:]
  del viewsSetNames[:]
  #DocumentType = request.forms.get('DocumentType')
  #ObjectName = request.forms.get('ObjectName')
  DocumentType="View"
  print DocumentType
  data = list(collection.find({"Type":DocumentType},{"_id":1,"Type":1,"FileTypes":1,"fileOIDs":1, "viewsSetNamesList":1, "viewSetList":1, "Name":1, "SceneName":1, "sensorType":1, "description":1}))
  print data
  jsonList = json.dumps(data,default=json_util.default)
  jdata = json.loads(jsonList)
  
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
  <p align="center">TABLE OF VIEWS</p>
  <tr>
    <th>Nr.</th>
    <th>View Name</th>
    <th>Sensor</th>
    <th>Files Number</th>
    <th>Date</th>
    <th>Scene</th>
    <th>Views Set</th>
    <th>Description</th>		
    <th>Read file</th>
    <th>Add file to view</th>
    <th>Remove Files</th>
  </tr>'''
  
  #if DocumentType=="Model":  
    #table =  '''   <style>
  #table, th, td {
      #border: 1px solid black;
      #padding: 5px;
  #}
  #table {
      #border-spacing: 15px;
  #}
  #</style>
  #<table style="width:50%">
  #<tr>
    #<th>Model name</th>
    #<th>Description</th>		
    #<th>ID</th>
  #</tr>'''
  
  #typeName="Name"
  
  if DocumentType=="View":
      typeName="Name"
  #if NodeName=="Model":
      #typeName="Name"
  
 
  date = "12-12-2014"
  fileOIDs="[]"
  i=0
  scene=""
  filesInView=False
  TypesList = ''
  existTypesList = []
  RemoveList = ''
  addString = ''
  for d in jdata:	# for all views
      for key, value in d.iteritems():	# for all keys
	    print "key value"
	    print key, value
	    if key==typeName:
		name=value
	    if key=="description":
		description=value 
	    if key=="sensorType":
		sensor=value
	    if key=="_id":
		_id=str(value)
	    if key=="SceneName":
		scene=value
		print scene
	    if key=="viewSetList":
		viewsSetL=value
		for d22 in viewsSetL:
		  ##print d2
		  for key, value in d22.iteritems():
		      #print key
		      if key=="ViewsSetOID":
			viewsSet.append(str(value))
	    if key=="viewsSetNamesList":
		viewsSetN=value
		for d22 in viewsSetN:
		  for key, value in d22.iteritems():
		      if key=="Name":
			viewsSetNames.append(str(value))
	    if key=="fileOIDs":
		#print fileOIDs
		filesInView=True
		fileOIDs=value
		FileList.append([])
		for d2 in fileOIDs:
		  ##print d2
		  for key, value in d2.iteritems():
		      #print key
		      if key=="fileOID":
			## print value
			print i
			FileList[i].append(str(value))
			print "FileList"
			print FileList[i]
	    if key=="FileTypes":
		  #print fileOIDs
		  FileTypes=value
		  FileList.append([])
		  for d2 in FileTypes:
		    ##print d2
		    for key, value in d2.iteritems():
			#print key
			if key=="Type":
			    existTypesList.append(str(value))
			    print value
			    print 'existTypesList:'
			    print existTypesList
			    TypesList=TypesList+'<OPTION selected label="'+value+'" value="'+value+'">'+value+'</OPTION>'
			    print TypesList
      removeList=[]
      exist=False
      if existTypesList==[]:
	removeList=fileTypesList
      else: 
	print existTypesList
	for fileType in fileTypesList:
	  for existType in existTypesList:
	    print fileType
	    print existType
	    if fileType==existType:
	      exist=True
	      break;
	  if exist==False:	
	    removeList.append(fileType)
	  exist=False
      print "licznik: "
      print i
      print "Remove List"
      print removeList
      addString = ''
      for fileType in removeList:
	addString = addString + '<OPTION selected label="'+fileType+'" value="'+fileType+'">'+fileType+'</OPTION>'
      filesNumber=0
      print 'AddString:'
      print addString
      if(filesInView==True):
      	filesNumber = len(FileList[i])
      table = table + addRowToView(i, name, sensor, filesNumber, date, scene, viewsSetNames, description, TypesList, addString)
      existTypesList = []
      addString=''
      TypesList=''
      del viewsSetNames[:]
      print "Zwiekszam licznik"
      if(filesInView==True):      	
	i=i+1
      	filesInView=False
  table = table + " </table>"
  insertButton = drawAddButton("empty","empty","empty", "full", addString)
  table = table + insertButton
  print FileList

  return table 
        
@get('/static/objects/views/addButton/<Name>/<DocumentType>/<SensorType>/<mode>/<addString>')
def drawAddButton(Name, DocumentType, SensorType, mode, addString):
  if mode=="full":
    addString = '''<OPTION selected label="ImageRgb" value="ImageRgb">ImageRgb</OPTION><OPTION selected label="FileCameraInfo" value="FileCameraInfo">FileCameraInfo</OPTION><OPTION selected label="ImageXyz" value="ImageXyz">ImageXyz</OPTION><OPTION selected label="ImageDepth" value="ImageDepth">ImageDepth</OPTION><OPTION selected label="ImageIntensity" value="ImageIntensity">ImageIntensity</OPTION><OPTION selected label="ImageMask" value="ImageMask">ImageMask</OPTION><OPTION selected label="StereoLeft" value="StereoLeft">StereoLeft</OPTION><OPTION selected label="StereoRight" value="StereoRight">StereoRight</OPTION><OPTION selected label="StereoLeftTextured" value="StereoLeftTextured">StereoLeftTextured</OPTION><OPTION selected label="StereoRightTextured" value="StereoRightTextured">StereoRightTextured</OPTION><OPTION selected label="PCXyz" value="PCXyz">PCXyz</OPTION><OPTION selected label="PCXyzRgb" value="PCXyzRgb">PCXyzRgb</OPTION><OPTION selected label="PCXyzSift" value="PCXyzSift">PCXyzSift</OPTION><OPTION selected label="PCXyzRgbSift" value="PCXyzRgbSift">PCXyzRgbSift</OPTION><OPTION selected label="PCXyzShot" value="PCXyzShot">PCXyzShot</OPTION><OPTION selected label="PCXyzRgbNormal" value="PCXyzRgbNormal">PCXyzRgbNormal</OPTION>'''
    return '''<p><form action='/static/objects/addfilecreate' method="post">
	  CREATE VIEW OR MODEL AND ADD FILE <br>
	  Full path: <input name="path" type="text" /><br>
	  Name: <input name="Name" type="text" /><br>
	  DocumentType: <input name="DocumentType" type="text" /><br>
	  ViewsSet: <input name="ViewsSet" type="text" /><br>
	  SensorType: <SELECT name="SensorType">
	      <OPTGROUP label="ReqFiletype">
		<OPTION selected label="Kinect1" value="Kinect1">Kinect1</OPTION>
		<OPTION label="Kinect2" value="Kinect2">Kinect2</OPTION>
		<OPTION label="Stereo" value="Stereo">Stereo</OPTION>
	      </OPTGROUP>
	  </SELECT>
	  Scene Name: <input name="scene" type="text" /><br>
	  Filetype: <SELECT name="ReqFiletype">
	      <OPTGROUP label="ReqFiletype">'''+addString+'''
	      </OPTGROUP>
	  </SELECT>
	  <input value="Add File" type="submit" />
      </form></p>
  '''
  else:
    return '''<p><form action='/static/objects/addfile/%s/%s/%s' method="post">
	    Full path: <input name="path" type="text" /><br>
	    Filetype: <SELECT name="ReqFiletype">
		<OPTGROUP label="ReqFiletype">'''%(Name, DocumentType, SensorType)+addString+'''
		</OPTGROUP>
	    </SELECT>
	    <input value="Add File" type="submit" />
	</form></p>
    '''

@get('/static/objects/views/removeButton/<Name>/<DocumentType>/<SensorType>/<mode>')
def drawRemoveButton(Name, DocumentType, SensorType, mode, i, TypesList):
  print "DocumentType :" 
  print DocumentType
  if mode=="full":
    return '''<p><form action='/static/objects/removefile' method="post">
	  Choose type of removed file<br>
	  Name: <input name="Name" type="text" /><br>
	  DocumentType: <input name="Type" type="text" /><br>
	  ViewsSet: <input name="ViewsSet" type="text" /><br>
	  SensorType: <SELECT name="SensorType">
	      <OPTGROUP label="ReqFiletype">'''+  TypesList+'''
	      </OPTGROUP>
	  </SELECT>
	  Scene Name: <input name="scene" type="text" /><br>
	  Filetype: <SELECT name="ReqFiletype">
	      <OPTGROUP label="ReqFiletype">'''+TypesList+'''
	      </OPTGROUP>
	  </SELECT>
	  <input value="Remove File" type="submit" />
      </form></p>
  '''
  else:
	  return '''<p><form action='/static/objects/removefile/%s/%s/%s' method="post">
		Choose type of removed file: <SELECT name="ReqFiletype">
		      <OPTGROUP label="ReqFiletype">'''%(Name, DocumentType, SensorType)+TypesList+'''
		      </OPTGROUP>
		  </SELECT>
		  <input value="Remove File" type="submit" />
	      </form></p>
	  '''
      
@route('/static/objects/models')
def showModelsInfo():
  print  "showModelsInfo"
  print "delete List"
  del FileList[:]
  del viewsSet[:]
  del viewsSetNames[:]
  DocumentType="Model"
  print DocumentType
  data = list(collection.find({"Type":DocumentType},{"_id":1,"Type":1,"FileTypes":1,"fileOIDs":1, "viewsSetNamesList":1, "viewSetList":1, "Name":1, "description":1}))
  print data
  jsonList = json.dumps(data,default=json_util.default)
  jdata = json.loads(jsonList)
  
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
  <p align="center">TABLE OF MODELS</p>
  <tr>
    <th>Nr.</th>
    <th>Model Name</th>
    <th>Files Number</th>
    <th>Date</th>
    <th>Views Set</th>
    <th>Description</th>		
    <th>Read file</th>
    <th>Add file to model</th>
    <th>Remove Files</th>
  </tr>'''
  
  
  if DocumentType=="Model":
      typeName="Name"
 
  date = "12-12-2014"
  fileOIDs="[]"
  i=0
  scene=""
  filesInModel=False
  TypesList = ''
  existTypesList = []
  RemoveList = ''
  addString = ''
  for d in jdata:	# for all views
      for key, value in d.iteritems():	# for all keys
	    print "key value"
	    print key, value
	    if key==typeName:
		name=value
	    if key=="description":
		description=value 
	    if key=="_id":
		_id=str(value)
	    if key=="viewSetList":
		viewsSetL=value
		for d22 in viewsSetL:
		  ##print d2
		  for key, value in d22.iteritems():
		      #print key
		      if key=="ViewsSetOID":
			viewsSet.append(str(value))
	    if key=="viewsSetNamesList":
		viewsSetN=value
		for d22 in viewsSetN:
		  for key, value in d22.iteritems():
		      if key=="Name":
			viewsSetNames.append(str(value))
	    if key=="fileOIDs":
		#print fileOIDs
		filesInModel=True
		fileOIDs=value
		FileList.append([])
		for d2 in fileOIDs:
		  ##print d2
		  for key, value in d2.iteritems():
		      #print key
		      if key=="fileOID":
			## print value
			print i
			FileList[i].append(str(value))
			print "FileList"
			print FileList[i]
	    if key=="FileTypes":
		  #print fileOIDs
		  FileTypes=value
		  FileList.append([])
		  for d2 in FileTypes:
		    ##print d2
		    for key, value in d2.iteritems():
			#print key
			if key=="Type":
			    existTypesList.append(str(value))
			    print value
			    print 'existTypesList:'
			    print existTypesList
			    TypesList=TypesList+'<OPTION selected label="'+value+'" value="'+value+'">'+value+'</OPTION>'
			    print TypesList
      removeList=[]
      exist=False
      if existTypesList==[]:
	removeList=fileTypesList
      else: 
	print existTypesList
	for fileType in fileTypesList:
	  for existType in existTypesList:
	    print fileType
	    print existType
	    if fileType==existType:
	      exist=True
	      break;
	  if exist==False:	
	    removeList.append(fileType)
	  exist=False
      print "licznik: "
      print i
      print "Remove List"
      print removeList
      addString = ''
      for fileType in removeList:
	addString = addString + '<OPTION selected label="'+fileType+'" value="'+fileType+'">'+fileType+'</OPTION>'
      filesNumber=0
      print 'AddString:'
      print addString
      if(filesInModel==True):
      	filesNumber = len(FileList[i])
      table = table + addRowToModel(i, name, filesNumber, date, viewsSetNames, description, TypesList, addString)
      existTypesList = []
      addString=''
      TypesList=''
      del viewsSetNames[:]
      print "Zwiekszam licznik"
      if(filesInModel==True):      	
	i=i+1
      	filesInModel=False
  table = table + " </table>"
  insertButton = drawAddButton("empty","empty","empty", "full", addString)
  table = table + insertButton
  print FileList
  return table 

def addRowToView(i, name, sensor, filesNumber, date, scene, viewsSet, description, TypesList, addString):
  print "addRow"
  # get file types and dynamically
  getButton = '''<form action='/static/img/gridfs/image/%s' method="post">
	    <SELECT name="ReqFiletype">
		<OPTGROUP label="ReqFiletype">
		      '''%(str(i))+TypesList+'''
		</OPTGROUP>
	    </SELECT>
	    <input value="Read File" type="submit" />
	</form>
    '''
  for item in viewsSet:
    print "ViewsSetOID"
    print item
    
   #ImageDepth
  insertButton = drawAddButton(name,"View",sensor, "exist", addString)
  removeButton = drawRemoveButton(name,"View",sensor, "exist", i, TypesList)

  return '''  <tr>
    <td>%s</td>
    <td>%s</td>
    <td>%s</td>		
    <td>%s</td>
    <td>%s</td>
    <td>%s</td>		
    <td>%s</td>
    <td>%s</td>
    <td>%s</td>	
    <td>%s</td>	
    <td>%s</td>	
    </tr>    '''%(str(i), name, sensor, filesNumber, date, scene, viewsSet, description, getButton, insertButton, removeButton)

def addRowToModel(i, name, filesNumber, date, viewsSet, description, TypesList, addString):
  print "addRow"
  # get file types and dynamically
  getButton = '''<form action='/static/img/gridfs/image/%s' method="post">
	    <SELECT name="ReqFiletype">
		<OPTGROUP label="ReqFiletype">
		      '''%(str(i))+TypesList+'''
		</OPTGROUP>
	    </SELECT>
	    <input value="Read File" type="submit" />
	</form>
    '''
  for item in viewsSet:
    print "ViewsSetOID"
    print item
    
   #ImageDepth
  insertButton = drawAddButton(name,"Model",'sensor', "exist", addString)
  removeButton = drawRemoveButton(name,"Model",'sensor', "exist", i, TypesList)

  return '''  <tr>
    <td>%s</td>
    <td>%s</td>
    <td>%s</td>		
    <td>%s</td>
    <td>%s</td>
    <td>%s</td>		
    <td>%s</td>
    <td>%s</td>
    <td>%s</td>	
    </tr>    '''%(str(i), name, filesNumber, date, viewsSet, description, getButton, insertButton, removeButton)
    
@route('/static/img/gridfs/getimage/<_id>/<contentType>/<extension>')
def get_img(_id,contentType,extension):
    dbname = 'containers'
    db = connection[dbname]
    fs = gridfs.GridFS(db)
    oid = ObjectId(str(_id))
    #print oid
    #gridout = fs.get_last_version(filename=filename)
    gridout = fs.get(oid)
    response.content_type = contentType+"/"+extension
    return gridout

@route('/static/img/documents/getimage/<_id>/<contentType>/<extension>')
def retrieve_image(_id,contentType,extension):
    oid = ObjectId(str(_id))
    document = containers.find_one({"_id":oid})
    filename = document["Name"]
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

@post('/static/objects/addfilecreate')
def add_file_create():
  absolutePath = request.forms.get('path')
  fileType = request.forms.get('ReqFiletype')
  Name = request.forms.get('Name')
  ViewsSet = request.forms.get('ViewsSet')
  scene = request.forms.get('scene')
  sensorType = request.forms.get('SensorType')
  DocumentType = request.forms.get('DocumentType')
  print "Path :" 
  print absolutePath
  fn = absolutePath[absolutePath.rfind("/")+1:]
  print "filename: "
  print fn
  print "DocumentType: "
  print DocumentType
  print "Name: "
  print Name
  print "fileType: "
  print fileType
  print "ViewsSet: "
  print ViewsSet

  # TODO add contentType dynamically
  
  print "Add file"
  print os.path.getsize(absolutePath)
  dbname = 'containers'
  db = connection[dbname]
  fs = gridfs.GridFS(db)
  fileID = fs.put( open(absolutePath, 'rb'), filename=fn )
  out = fs.get(fileID)
  print out.length
  print out.filename
  ts = str(int(round(time.time() * 1000)))
  fileDocument = ''
  if DocumentType=="View":
    fileDocument = {"_id" : fileID, "filename" : ts+fn,  "Name" : ts+fn, "Type" : "File", "ViewName" : Name, "size" : out.length , "place" : "grid", "fileType" : fileType, "contentType" : "image/png"}
  if DocumentType=="Model":
    fileDocument = {"_id" : fileID, "filename" : ts+fn, "Name" : ts+fn, "Type" : "File", "ModelName" : Name, "size" : out.length , "place" : "grid", "fileType" : fileType, "contentType" : "image/png"}
  file_doc_id = collection.insert(fileDocument)
  post_id = 0
  ReqFiletype = request.forms.get('ReqFiletype')
  document = collection.find_one({"Type": DocumentType, "Name" : Name},{"_id":1,"Type":1,"FileTypes":1, "Name":1})
  print document
  scene_id = 0000000000000000000
  if document==None:
    print "No documents!!!"
    ## create view document if not exist
    if DocumentType=='View':
      scene_id = create_scene(scene)
    viewID = create_document(DocumentType, Name, sensorType, scene)
    if DocumentType=='View':
      update_scene_document(scene, DocumentType, viewID)
    viewsset = ViewsSet.split(';')
    for item in viewsset:
      ViewsSetDoc = collection.find_one({"Type": "ViewsSet", "viewsSetNamesList" : item})
      if ViewsSetDoc==None:
	viewsSetID = create_viewSetdocument(item)
      else:
	viewsSetID=ViewsSetDoc["_id"];
      update_viewormodel_document(Name, DocumentType, scene, scene_id, item, viewsSetID)
      if DocumentType=="View":
	update_viewSetdocument(viewID, viewsSetID)
  else:
    print "Document founded!!!"
   ## update view document
  update_document(fileID, Name, DocumentType, fileType)
  return '''File inserted successfully'''


@post('/static/objects/removefile/<Name>/<DocumentType>/<sensorType>')
def remove_file(Name,DocumentType,sensorType):
  fileType = request.forms.get('ReqFiletype')
  sensorType = request.forms.get('SensorType')
  print "DocumentType: "
  print DocumentType
  print "Name: "
  print Name
  print "fileType: "
  print fileType

  dbname = 'containers'
  db = connection[dbname]
  document = ''
  if DocumentType=="View":
    document = collection.find_one({"Type" : "File", "fileType" : fileType, "ViewName" : Name},{"_id":1,"fileType":1,"Type":1, "Name":1})
  if DocumentType=="Model":
    document = collection.find_one({"Type" : "File", "fileType" : fileType, "ModelName" : Name},{"_id":1,"fileType":1,"Type":1, "Name":1})
  print document
  oid  = document["_id"]
  if DocumentType=="View":
    collection.update({"Type": "View", "Name" : Name}, {"$pull" :{"FileTypes" :{"Type":fileType}}});
    collection.update({"Type": "View", "Name" : Name}, {"$pull" :{"fileOIDs" :{"fileOID":str(oid)}}});
  if DocumentType=="Model":
    collection.update({"Type": "Model", "Name" : Name}, {"$pull" :{"FileTypes" :{"Type":fileType}}});
    collection.update({"Type": "Model", "Name" : Name}, {"$pull" :{"fileOIDs" :{"fileOID":str(oid)}}});
  #remove document
  collection.remove(oid)
  return '''File removed successfully'''

def create_viewSetdocument(ViewsSet):
  print "create ViewsSet document"  
  post = { "Type" : "ViewsSet", "viewsSetNamesList" : ViewsSet, "description" : "" } 
  viewsSetID = collection.insert(post)
  return viewsSetID 
	
def update_viewSetdocument(ViewOID, viewsSetID):
  print "update ViewsSet document"
  collection.update({"_id": viewsSetID}, { "$addToSet":{"ViewsList":{"viewOID": str(ViewOID)}}}, upsert=True)


def update_scene_document(Name, DocumentType, viewOID):
  print "update"
  collection.update({"Type": "Scene", "Name" : Name}, { "$addToSet":{"ViewsList":{ "viewOID" :  str(viewOID)}} }, upsert=True)

def update_viewormodel_document(Name, DocumentType, sceneName, sceneOID, ViewsSet, viewsSetID):
  print "update"
  if DocumentType=="View":
    collection.update({"Type": DocumentType, "Name" : Name}, { "$set":{"SceneOID" :  str(sceneOID)} }, upsert=True)
    collection.update({"Type": DocumentType, "Name" : Name}, { "$set":{"SceneName" : sceneName }}, upsert=True)
  collection.update({"Type": DocumentType, "Name" : Name}, { "$addToSet":{"viewSetList" : {"ViewsSetOID" : str(viewsSetID)}}}, upsert=True)
  collection.update({"Type": DocumentType, "Name" : Name}, { "$addToSet":{"viewsSetNamesList":{"Name": ViewsSet}}}, upsert=True)

  
def create_scene(scene):
    print "create scene document"  
    post = { "Type" : "Scene", "Name" : scene, "description" : "" } 
    post_id = collection.insert(post)
    return post_id 

@post('/static/objects/addfile/<Name>/<DocumentType>/<sensorType>')
def add_file(Name,DocumentType,sensorType):
  absolutePath = request.forms.get('path')
  fileType = request.forms.get('ReqFiletype')
  sensorType = request.forms.get('sensorType')
  print "Path :" 
  print absolutePath
  fn = absolutePath[absolutePath.rfind("/")+1:]
  print "filename: "
  print fn
  print "DocumentType: "
  print DocumentType
  print "Name: "
  print Name
  print "fileType: "
  print fileType
  # TODO add contentType dynamically
  
  print "Add file"
  print os.path.getsize(absolutePath)
  dbname = 'containers'
  db = connection[dbname]
  fs = gridfs.GridFS(db)
  fileID = fs.put( open(absolutePath, 'rb'), filename=fn )
  out = fs.get(fileID)
  print out.length
  ts = str(int(round(time.time() * 1000)))
  fileDocument = ''
  if DocumentType=="View":
    fileDocument = {"_id" : fileID,  "filename" : ts+fn, "Name" : ts+fn, "Type" : "File", "ViewName" : Name, "size" :out.length , "place" : "grid", "fileType" : fileType, "contentType" : "image/png"}
  if DocumentType=="Model":
    fileDocument = {"_id" : fileID,  "filename" : ts+fn, "Name" : ts+fn, "Type" : "File", "ModelName" : Name, "size" :out.length , "place" : "grid", "fileType" : fileType, "contentType" : "image/png"}
  
  file_doc_id = collection.insert(fileDocument)
  post_id = 0
  ReqFiletype = request.forms.get('ReqFiletype')
  document = collection.find_one({"Type": DocumentType, "Name" : Name},{"_id":1,"Type":1,"FileTypes":1, "Name":1})
  print document
  if document==None:
    print "No documents!!!"
    # create view document if not exist
    create_document(DocumentType, Name, sensorType)
  else:
    print "Document founded!!!"
  # update view document
  update_document(fileID, Name, DocumentType, fileType)
  return '''File inserted successfully'''

def create_document(DocumentType, Name, sensorType, scene):
  print "create document"  
  if DocumentType=="View":
    post = { "Type" : DocumentType, "Name" : Name, "description" : "", "sensorType" : sensorType, "SceneName" : scene }
  if DocumentType=="Model":
    post = { "Type" : DocumentType, "Name" : Name, "description" : ""}
  post_id = collection.insert(post)
  return post_id
 
def update_document(fileID,Name, DocumentType, fileType):
  print "update"
  collection.update({"Type": DocumentType, "Name" : Name}, { "$addToSet":{"fileOIDs":{ "fileOID" :  str(fileID)}} }, upsert=True)
  collection.update({"Type": DocumentType, "Name" : Name}, { "$addToSet":{"FileTypes":{ "Type" : fileType }} }, upsert=True)

@route('/static/img/gridfs/image/<i>', method='POST')
def view_images(i):
    print FileList
    # zabezpieczyc na wypadek nie znalezienia niczego
    ReqFiletype = request.forms.get('ReqFiletype')
    print "view_images"
    print "FileList[i]:"
    k=int(i)
    print k
    print FileList[k]
    lenght = len(FileList[k])
    #print lenght
    #print str(FileList)
    #print ', '.join(FileList)
    foto2=""
    fileType=""
    print "for"
    for item in FileList[k]:
      print item
      oid = ObjectId(str(item))
      print oid
      res = list(collection.find({"_id":oid},{"Name":1, "size":1, "place":1, "extension":1, "contentType":1, "fileType":1}))
      print res
      jsonList2 = json.dumps(res,default=json_util.default)
      jdata2 = json.loads(jsonList2)
      for d2 in jdata2:
	for key2, value2 in d2.iteritems():
	  if key2=="Name":
	    filename=value2
	  if key2=="fileType":
	    fileType=value2
	  if key2=="contentType":
	    contentType=value2
	  if key2=="place":
	    place=value2
	#    print place
      if ReqFiletype==fileType:
	    if place=="grid":
		    _id = str(oid)
		    path = "/static/img/gridfs/getimage/"+_id+"/"+contentType;
		    foto = "<p> Image %s:</p>  <p><img src=%s></p>" %(filename,path)
	    if place=="document":
		    document = collection.find_one({"_id":oid})
		    filename = document["Name"]
		    data1 = json.loads(dumps(document))
		    img = data1
		    img1 = img[filename]
		    img2 = json.loads(dumps(img1))
		    print img2.get("$type",{})
		    binary =  img2.get("$binary",{})
		    decode = binary.decode()
		    foto =  '<p> Image %s:</p>  <p>  <img alt="sample" src="data:image/png;base64,{0}"></p>'.format(decode)%(filename)
	    foto2=foto2+foto	
    print "delete List"
 #   del FileList[:]
    return foto2
    
run(host='localhost', port=8080)
