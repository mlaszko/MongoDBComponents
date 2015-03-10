/*
 * Document.hpp
 *
 *  Created on: Feb 20, 2015
 *      Author: lzmuda
 */

#ifndef DOCUMENT_HPP_
#define DOCUMENT_HPP_

namespace MongoDB {
using namespace mongo;

class Document
{
protected:
	std::string Type;
	OID oid;
	std::string Name;
	BSONObj BSONDocument;
	std::string dateOfInsert;
	std::string description;
public:
	 //BSONObj getDocument();
	 Document();
	 //Document(std::string& Name, std::string& type) : Name(Name), Type(type){};
	 Document(std::string& Name) : Name(Name){};
	 Document(mongo::OID& oid) : oid(oid){Name = std::string("");};
};

}
#endif /* DOCUMENT_HPP_ */
