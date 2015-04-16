#-*- coding: utf-8 -*-
from web_application import *
import os
import sys
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-d", "--dir", help="katalog z plikami", type=str)
parser.add_argument("-s", "--scene", help="nazwa sceny", type=str)
parser.add_argument("-st", "--sensorType", help="sensorType", default='Kinect1', type=str)
args = parser.parse_args()

if(not args.dir):
    print "Podaj katalog -d katalog"
    sys.exit(0)
if(not args.scene):
    print "Podaj nazwę sceny -s scena"
    sys.exit(0)

dir = args.dir
scene = args.scene
sensorType = args.sensorType
DocumentType = "View"

#tworzy widok i dodaje do niego pierwszy plik
def addFileCreate(path, name, file_type):
  absolutePath = path
  fileType = file_type
  Name = name
  ViewsSet = " "
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
  fileDocument = {"_id" : fileID, "filename" : ts+fn,  "Name" : ts+fn, "Type" : "File", "ViewName" : Name, "size" : out.length , "place" : "grid", "fileType" : fileType, "contentType" : "image/png"}
  file_doc_id = collection.insert(fileDocument)
  post_id = 0
  ReqFiletype = fileType
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
  print '''File inserted successfully'''


#dodaje plik do widoku
def addFile(path, Name, file_type):
  absolutePath = path
  fileType = file_type
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

  print "Add file"
  print os.path.getsize(absolutePath)
  dbname = 'containers'
  db = connection[dbname]
  fs = gridfs.GridFS(db)
  fileID = fs.put( open(absolutePath, 'rb'), filename=fn )
  out = fs.get(fileID)
  print out.length
  ts = str(int(round(time.time() * 1000)))
  fileDocument = {"_id" : fileID,  "filename" : ts+fn, "Name" : ts+fn, "Type" : "File", "ViewName" : Name, "size" :out.length , "place" : "grid", "fileType" : fileType, "contentType" : "image/png"}

  file_doc_id = collection.insert(fileDocument)
  post_id = 0
  ReqFiletype = fileType
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
  print '''File inserted successfully'''





#obrazy muszą być w kolejności rgb, depth, rgb, depth, ...

name = ""
for filename in os.listdir(dir):
        if os.path.isdir(filename) == False:
                if filename.find("rgb.png")>0:
                    file_type = "ImageRgb"
                    name = filename.rstrip("_rgb.png")  #te same name dla pliku depth
                    addFileCreate(dir+"/"+filename, name, file_type)
                elif filename.find("depth.png")>0:
                    file_type = "ImageDepth"
                    addFile(dir+"/"+filename, name, file_type)
