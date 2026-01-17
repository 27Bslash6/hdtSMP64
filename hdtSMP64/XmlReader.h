#pragma once

#include "hdtSkinnedMesh/hdtBulletHelper.h"

#include "XmlInspector/CharactersReader.hpp"
#include "XmlInspector/XmlInspector.hpp"

namespace hdt
{
	class XMLReader : public Xml::Inspector<Xml::Encoding::Utf8Writer>
	{
		typedef Inspector<Xml::Encoding::Utf8Writer> Base;
		bool isEmptyStart;

	public:
		XMLReader(BYTE* data, size_t count) : Base(data, data + count) {}

		typedef Xml::Inspected Inspected;

		bool Inspect();
		Xml::Inspected GetInspected();

		void skipCurrentElement();
		void nextStartElement();

		bool hasAttribute(const std::string& name);
		std::string getAttribute(const std::string& name);
		std::string getAttribute(const std::string& name, const std::string& def);

		float getAttributeAsFloat(const std::string& name);
		int getAttributeAsInt(const std::string& name);
		bool getAttributeAsBool(const std::string& name);

		std::string readText();
		float readFloat();
		int readInt();
		bool readBool();

		btVector3 readVector3();
		btQuaternion readQuaternion();
		btQuaternion readAxisAngle();
		btTransform readTransform();
	};
} // namespace hdt
