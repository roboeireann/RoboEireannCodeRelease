/**
 * @file InStreams.cpp
 *
 * Implementation of in stream classes.
 *
 * @author Thomas Röfer
 * @author Martin Lötzsch
 */

#include <cstring>
#include <cstdlib>
#include <cstdio>

#include "InStreams.h"
#include "Platform/BHAssert.h"

void StreamReader::skipData(size_t size, PhysicalInStream& stream)
{
  // default implementation
  char* dummy = new char[size];
  readData(dummy, size, stream);
  delete[] dummy;
}

void PhysicalInStream::skipInStream(size_t size)
{
  // default implementation
  char* dummy = new char[size];
  readFromStream(dummy, size);
  delete[] dummy;
}

void InText::readString(std::string& value, PhysicalInStream& stream)
{
  value = "";
  skipWhitespace(stream);
  bool containsSpaces = theChar == '"';
  if(containsSpaces && !isEof(stream))
    nextChar(stream);
  while(!isEof(stream) && (containsSpaces || !isWhitespace()) && (!containsSpaces || theChar != '"'))
  {
    if(theChar == '\\')
    {
      nextChar(stream);
      if(theChar == 'n')
        theChar = '\n';
      else if(theChar == 'r')
        theChar = '\r';
      else if(theChar == 't')
        theChar = '\t';
    }
    value += theChar;
    if(!isEof(stream))
      nextChar(stream);
  }
  if(containsSpaces && !isEof(stream))
    nextChar(stream);
  skipWhitespace(stream);
}

void InText::readData(void* p, size_t size, PhysicalInStream& stream)
{
  for(size_t i = 0; i < size; ++i)
    readChar(*reinterpret_cast<char*&>(p)++, stream);
}

bool InText::isWhitespace()
{
  return theChar == ' ' || theChar == '\n' || theChar == '\r' || theChar == '\t';
}

void InText::skipWhitespace(PhysicalInStream& stream)
{
  while(!isEof(stream) && isWhitespace())
    nextChar(stream);
}

void InText::readBool(bool& value, PhysicalInStream& stream)
{
  skipWhitespace(stream);
  if(!isEof(stream))
  {
    if(theChar == '0' || theChar == '1')
    {
      value = theChar != '0';
      nextChar(stream);
    }
    else
    {
      value = theChar != 'f';
      static const char* falseString = "false";
      static const char* trueString = "true";
      const char* p = value ? trueString : falseString;
      while(!isEof(stream) && *p && theChar == *p)
      {
        ++p;
        nextChar(stream);
      }
    }
  }
}

void InText::readChar(char& d, PhysicalInStream& stream)
{
  int i;
  readInt(i, stream);
  d = static_cast<char>(i);
}

void InText::readSChar(signed char& d, PhysicalInStream& stream)
{
  int i;
  readInt(i, stream);
  d = static_cast<signed char>(i);
}

void InText::readUChar(unsigned char& d, PhysicalInStream& stream)
{
  unsigned u;
  readUInt(u, stream);
  d = static_cast<unsigned char>(u);
}

void InText::readShort(short& d, PhysicalInStream& stream)
{
  int i;
  readInt(i, stream);
  d = static_cast<short>(i);
}

void InText::readUShort(unsigned short& d, PhysicalInStream& stream)
{
  unsigned u;
  readUInt(u, stream);
  d = static_cast<unsigned short>(u);
}

void InText::readInt(int& d, PhysicalInStream& stream)
{
  skipWhitespace(stream);
  int sign = 1;
  if(!isEof(stream) && theChar == '-')
  {
    sign = -1;
    nextChar(stream);
  }
  else if(!isEof(stream) && theChar == '+')
    nextChar(stream);
  unsigned u;
  readUInt(u, stream);
  d = sign * static_cast<int>(u);
}

void InText::readUInt(unsigned int& d, PhysicalInStream& stream)
{
  buf = "";
  skipWhitespace(stream);
  while(!isEof(stream) && isdigit(theChar))
  {
    buf += theChar;
    nextChar(stream);
  }
  d = static_cast<unsigned>(strtoul(buf.c_str(), nullptr, 0));
  skipWhitespace(stream);
}

void InText::readFloat(float& d, PhysicalInStream& stream)
{
  double f;
  readDouble(f, stream);
  d = static_cast<float>(f);
}

void InText::readDouble(double& d, PhysicalInStream& stream)
{
  buf = "";
  skipWhitespace(stream);
  if(!isEof(stream) && (theChar == '-' || theChar == '+'))
  {
    buf += theChar;
    nextChar(stream);
  }
  while(!isEof(stream) && isdigit(theChar))
  {
    buf += theChar;
    nextChar(stream);
  }
  if(!isEof(stream) && theChar == '.')
  {
    buf += theChar;
    nextChar(stream);
  }
  while(!isEof(stream) && isdigit(theChar))
  {
    buf += theChar;
    nextChar(stream);
  }
  if(!isEof(stream) && (theChar == 'e' || theChar == 'E'))
  {
    buf += theChar;
    nextChar(stream);
  }
  if(!isEof(stream) && (theChar == '-' || theChar == '+'))
  {
    buf += theChar;
    nextChar(stream);
  }
  while(!isEof(stream) && isdigit(theChar))
  {
    buf += theChar;
    nextChar(stream);
  }
  d = atof(buf.c_str());
  skipWhitespace(stream);
}

bool InText::expectString(const std::string& str, PhysicalInStream& stream)
{
  const char* p = str.c_str();
  if(str.length())
  {
    while(*p)
    {
      if(isEof(stream) || theChar != *p)
        return false;
      ++p;
      nextChar(stream);
    }
  }
  return true;
}

void InMemory::readFromStream(void* p, size_t size)
{
  if(memory)
  {
    std::memcpy(p, memory, size);
    memory += size;
  }
}

void InMap::parse(In& stream, const std::string& name)
{
  map = new SimpleMap(stream, name);
  this->name = name;
  stack.reserve(20);
}

void InMap::printError(const std::string& msg)
{
  if(showErrors)
  {
    std::string path;
    for(const auto& entry : stack)
    {
      if(entry.key)
      {
        if(!path.empty())
          path += '.';
        path += entry.key;
      }
      else
        path += '[' + std::to_string(entry.type) + ']';
    }
    static_cast<void>(msg);
    FAIL(name << (name.empty() || path.empty() ? "" : ", ") <<
         path << (name.empty() && path.empty() ? "" : ": ") << msg);
  }
}

void InMap::inUInt(unsigned int& value)
{
  Entry& e = stack.back();
  if(e.type == -1)
  {
    if(e.value)
    {
      const auto* array = dynamic_cast<const SimpleMap::Array*>(e.value);
      if(array)
        value = static_cast<unsigned>(array->size());
      else
        printError("array expected");
    }
    else
      value = 0;
  }
  else
    in(value);
}

void InMap::read(void*, size_t)
{
  FAIL("Unsupported operation.");
}

void InMap::skip(size_t)
{
  FAIL("Unsupported operation.");
}

void InMap::select(const char* name, int type, const char* enumType)
{
  ASSERT(map);
  ASSERT(name || type >= 0);

  Streaming::trimName(name);
  const SimpleMap::Value* value = stack.empty() ? (const SimpleMap::Value*) *map : stack.back().value;
  if(!value) // invalid
    stack.emplace_back(name, nullptr, type, enumType); // add more invalid
  else if(type >= 0) // array element
  {
    const auto* array = dynamic_cast<const SimpleMap::Array*>(value);
    if(array)
    {
      if(type < static_cast<int>(array->size()))
        stack.emplace_back(name, (*array)[type], type, enumType);
      else
      {
        printError("array index out of range");
        stack.emplace_back(name, nullptr, type, enumType); // add invalid
      }
    }
    else
    {
      printError("array expected");
      stack.emplace_back(name, nullptr, type, enumType); // add invalid
    }
  }
  else // record element
  {
    const auto* record = dynamic_cast<const SimpleMap::Record*>(value);
    if(record)
    {
      auto i = record->find(name);
      if(i != record->end())
        stack.emplace_back(name, i->second, type, enumType);
      else
      {
        printError(std::string("attribute '") + name + "' not found");
        stack.emplace_back(name, nullptr, type, enumType); // add invalid
      }
    }
    else
    {
      printError("record expected");
      stack.emplace_back(name, nullptr, type, enumType); // add invalid
    }
  }
}

void InMap::deselect()
{
  stack.pop_back();
}

InMapMemory::InMapMemory(const void* memory, size_t size, bool showErrors) :
  InMap(showErrors),
  stream(memory, size)
{
  parse(stream);
}
