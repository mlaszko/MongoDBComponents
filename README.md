MongoDBComponents - DisCODe Component Library
===============================

Description
-----------

Contains components and scripts related to utilization of MongoDB as robot knowledge repository.

Dependencies
------------

OpenCV - version 2.4.8

PCL - version 1.7.1. 

CvCoreTypes - DisCODe DCL

PCLCoreTypes - DisCODe DCL

Utilization
----------

   * Run the MongoDB deamon from your mongodb directory (e.g /home/discode/mongo_db/mongodb-linux-x86_64-2.6.7/bin)

./mongod --rest --dbpath=/home/discode/mongo_db/wut_mongo_db

   * In order to enable the web browser content management you must additionally run the python script (located in MongoDBComponents/src/python directory in your discode workspace)

python web_application.py
    
   * Web browser addresses
   
http://localhost:28017/ - MongoDB status

http://localhost:8080/static/objects/models - models management

http://localhost:8080/static/objects/views - views management

Maintainer
----------

[Łukasz Żmuda](lukzmuda1@gmail.com)

[Tomasz Kornuta](tkornuta@gmail.com)
