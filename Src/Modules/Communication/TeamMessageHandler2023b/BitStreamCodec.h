/**
 * @file BitStreamCodec.h
 *
 * Encode or decode primitives and common composite types as bit streams
 * 
 * To keep things simple, we only implement the minimum necessary,
 * which is the ability to read and write bits, integers up to size unsigned
 * and floats (not doubles).
 * 
 * Collection types must be written manually by the caller. Just write a length
 * first (as an integer) and then the elements in a loop.
 *
 * @author Rudi Villing
 */

// #include "ByteBuffer.h"

#include "Tools/TextLogging.h"


using Byte = uint8_t;

namespace BitStream
{

struct TL
{
  static TextLogger& get()
  {
    static TextLogger& tlogger = TextLogging::get("BitStreamCodec", TextLogging::INFO);
    return tlogger;
  }
};
inline TextLogger& tlogger() { return TL::get(); }

/**
 * get the minimum number of bits needed to represent a number
 */
inline unsigned numBitsNeeded(unsigned value)
{
  // WARNING: this builtin counts the number of most significant zeros for
  // gcc and clang but other approaches are needed for other compilers
  return value > 0 ? (sizeof(unsigned) * 8 - __builtin_clz(value)) : 0;
}

class Writer
{
private:
  // DECL_TLOGGER_FN(tlogger, "BitStreamCodec::Writer", TextLogging::VERBOSE);

  TextLogger& tlogger = TextLogging::get("BitStreamCodec::Writer", TextLogging::INFO);

public:
  Writer(Byte *data, unsigned capacity) : mData(data), mCapacity(capacity) {}

  void writeBits(unsigned value, std::size_t nbits, const char* name=nullptr)
  {
    ASSERT(((mBitIndex + nbits) / 8) <= mCapacity);
    ASSERT(0 < nbits && nbits < (sizeof(value) * 8));

    TLOGV(tlogger, "writeBits: {:#32b}/{}{}", value & static_cast<unsigned>((1LL << nbits) - 1), nbits,
          name ? fmt::format(" ({})", name) : "");

    // write from LSbit of LSByte upwards
    // write the complete bytes first...
    for (; nbits >= 8; nbits -= 8)
    {
      writeBitsUnchecked(static_cast<uint8_t>(value & 0xFF), 8);
      value >>= 8;
    }
    // ...and then the remaining bits (if any)
    if (nbits)
      writeBitsUnchecked(static_cast<uint8_t>(value & 0xFF), nbits);
  }

  /**
   * write an integer using the specified number of bits after apply an offset and scaleFactor
   * (only for types up to sizeof(unsigned)).
   * 
   * Note that writeBits will write the least significant nbits, so you need to ensure that
   * nbits >= numBitsNeeded((maxVal - minVal) / scale)
   * 
   * Note also that minVal and maxVal are inclusive, i.e. maxVal is a legal value
   * rather than being limited to maxVal-1 if you are used to thinking about array sizes etc.
  */
  template <class T>
  void writeInteger(T value, T minVal, T maxVal, unsigned scale, unsigned nbits, const char* name=nullptr)
  {
    TLOGV(tlogger, "writeInteger: {}{}", value, name ? fmt::format(" ({})", name) : "");

    ASSERT(nbits > 0);

    const unsigned scaledValue = (std::max(minVal, std::min(maxVal, value)) - minVal) / scale;
    writeBits(scaledValue, nbits, name); // write nbits num least significant bits
  }

  /**
   * write a float using the specified params
  */
  template <class T>
  void writeFloat(T value, T minVal, T maxVal, unsigned nbits, const char* name=nullptr)
  {
    TLOGV(tlogger, "writeFloat: {}{}", value, name ? fmt::format(" ({})", name) : "");

    ASSERT(0 < nbits && nbits < sizeof(unsigned)*8);

    // note that the normalized value will also be non-negative (i.e. eventually suitable for unsigned)
    const double normalizedValue = (std::max(minVal, std::min(maxVal, value)) - minVal) / (maxVal - minVal);
    // mid tread quantization with limits to prevent overflowing the number of bits
    const unsigned maxQuantized = static_cast<unsigned>((1LL << nbits) - 1);
    const unsigned quantizedValue = static_cast<unsigned>((normalizedValue * maxQuantized) + 0.5);
    writeBits(std::min(maxQuantized, quantizedValue), nbits, name);
  }

  void finalizeBuffer()
  {
    if (mBitIndex % 8)
      writeBits(0, 8 - (mBitIndex % 8)); // zero out remaining bits of the final byte
  }

  void clear() { mBitIndex = 0; }

  int size() { return (mBitIndex + 7) / 8; }


private:
  Byte* mData;
  unsigned mCapacity;
  unsigned mBitIndex = 0; // bit index into data


  // write LSbit first 8 bits or less
  void writeBitsUnchecked(uint8_t value, std::size_t nbits)
  {
    for (std::size_t iBit = 0; iBit < nbits; iBit++, mBitIndex++)
    {
      // dest = data[byteOffset derived from mBitIndex]
      // first clear dest[bit mBitIndex],
      mData[mBitIndex / 8] &= ~(0x1 << (mBitIndex % 8));
      // then bitwiseOR value[bit i] left shifted to correct position for merging with dest[bit mBitIndex]
      mData[mBitIndex / 8] |= ((value >> iBit) & 0x1) << (mBitIndex % 8);
    }
  }
};



class Reader
{
private:
  TextLogger& tlogger = TextLogging::get("BitStreamCodec::Reader", TextLogging::INFO);

public:
  Reader(const Byte* data, unsigned size) : mData(data), mSize(size) {}

  void readBits(unsigned& value, std::size_t nbits, const char* name=nullptr)
  {
    ASSERT(((mBitIndex + nbits) / 8) <= mSize);
    ASSERT(nbits < (sizeof(value) * 8));

    // read into LSbit of LSByte upwards
    // read the complete bytes first...
    value = 0;
    unsigned ibit = 0;
    for (; (ibit + 8) <= nbits; ibit += 8)
    {
      value |= readBitsUnchecked(8) << ibit;
    }
    // ...and then the remaining bits
    if (ibit < nbits)
      value |= readBitsUnchecked(nbits - ibit) << ibit;

    TLOGV(tlogger, "readBits: {:#32b}/{}{}", value & ((1U << nbits) - 1), nbits, name ? fmt::format(" ({})", name) : "");
  }

  /**
   * read an integer using the specified number of bits after apply an offset and scaleFactor
   * (only for types up to sizeof(unsigned)) 
  */
  template <class T>
  void readInteger(T& value, T minVal, T /*maxVal*/, unsigned scale, unsigned nbits, const char* name=nullptr)
  {
    //(void)maxVal; // unused but in parameter list for consistency with other methods
    unsigned scaledValue = 0;
    readBits(scaledValue, nbits, name);
    value = static_cast<T>(scaledValue * scale) + minVal;

    TLOGV(tlogger, "readInteger: {}{}", value, name ? fmt::format(" ({})", name) : "");
  }

  /**
   * read a float using the specified params
   */
  template <class T>
  void readFloat(T& value, T minVal, T maxVal, unsigned nbits, const char* name=nullptr)
  {
    unsigned quantizedValue = 0;
    readBits(quantizedValue, nbits, name);
    value = static_cast<T>(quantizedValue / static_cast<double>((1LL << nbits) - 1) * (maxVal - minVal) + minVal);

    TLOGV(tlogger, "readFloat: {}{}", value, name ? fmt::format(" ({})", name) : "");
  }

  std::size_t numBytesRead() { return (mBitIndex + 7) / 8; }

private:
  const Byte* mData;
  unsigned mSize;
  unsigned mBitIndex = 0; // bit index into data


  // write LSbit first
  uint8_t readBitsUnchecked(std::size_t nbits)
  {
    uint8_t value = 0;
    for (std::size_t iBit = 0; iBit < nbits; iBit++, mBitIndex++)
    {
      // data[byteOffset] bitwiseOR with value[at bit i] left shifted to correct position for merging
      const unsigned bitIndexInByte = mBitIndex % 8;
      value |= (((mData[mBitIndex / 8] & (0x1 << bitIndexInByte)) >> bitIndexInByte) << iBit);
    }
    return value;
  }
};


template <class T>
struct Bits
{
  unsigned nbits;
  const char* name;

  Bits(const char* name, unsigned nbits) : nbits(nbits), name(name) { ASSERT(nbits > 0); }

  void write(Writer& w, T value) { w.writeBits(static_cast<unsigned>(value), nbits, name); }
  void read(Reader& r, T& value) { r.readBits(static_cast<unsigned&>(value), nbits, name); }

  bool checkQuantEqual(T v1, T v2)
  {
    const unsigned bits1 = v1 & static_cast<T>((1LL << nbits) - 1);
    const unsigned bits2 = v2 & static_cast<T>((1LL << nbits) - 1);
    return bits1 == bits2;
  }
};



template <class T>
struct Integer
{
  T minVal;
  T maxVal;
  unsigned scale;
  unsigned nbits;
  const char* name;

  Integer(const char *name, T minVal, T maxVal, unsigned scale = 1, unsigned nbitsIn = 0)
      : minVal(minVal), maxVal(maxVal), scale(scale), nbits(nbitsIn), name(name)
  {
    ASSERT(minVal < maxVal);

    if (nbits == 0)
    {
      nbits = numBitsNeeded(static_cast<unsigned>(((maxVal - minVal) / scale)));
      TLOGD(tlogger(), "Integer: max-min = {}, scale {}, nbits = {}", maxVal - minVal, scale, nbits);
    }

    ASSERT(nbits > 0);
  }

  void write(Writer& w, T value) { w.writeInteger(value, minVal, maxVal, scale, nbits, name); }
  void read(Reader& r, T& value) { r.readInteger(value, minVal, maxVal, scale, nbits, name); }

  bool checkQuantEqual(T v1, T v2)
  {
    const unsigned scaled1 = (std::max(minVal, std::min(maxVal, v1)) - minVal) / scale;
    const unsigned scaled2 = (std::max(minVal, std::min(maxVal, v2)) - minVal) / scale;
    return scaled1 == scaled2;
  }
};

/// Contiguous macro based enum from zero to numOfValues (= maxValidValue + 1)
template <class T>
struct ContiguousEnum
{
  Integer<unsigned> integer;

  ContiguousEnum(const char* name, T numOfValues) : integer(name, 0, numOfValues-1) {}

  void write(Writer& w, T value) { integer.write(w, int(value)); }
  void read(Reader &r, T &value)
  {
    unsigned inVal;
    integer.read(r, inVal);
    value = static_cast<T>(inVal);
  }
};



template <class T>
struct Float
{
  T minVal;
  T maxVal;
  unsigned nbits;
  T resolution;
  const char *name;

  Float(const char *name, T minVal, T maxVal, unsigned nbits)
      : minVal(minVal), maxVal(maxVal), nbits(nbits), name(name)
  {
    ASSERT(nbits > 0);
    resolution = maxVal - minVal / ((1LL << nbits) - 1);
  }

  void write(Writer &w, T value, T altValue = 0)
  {
    w.writeFloat(std::isfinite(value) ? value : altValue, minVal, maxVal, nbits, name);
  }

  void read(Reader& r, T& value) { r.readFloat(value, minVal, maxVal, nbits, name); }

  bool checkQuantEqual(T value1, T value2)
  {
    return std::fabs(value1 - value2) < (resolution * 1.5f);
  }
};

using Floatf = Float<float>;
using Timestamp = Integer<unsigned>;

/**
 * timestamp relative to some reference using linear quantised value
 * 
 * The timestamp may be before or after the reference time.
 * Relative timestamps in the past will be represented by negative numbers
 * whereas those in the future will be represented by positive numbers
*/
struct RelativeTimestamp
{
  enum RefType { beforeRef, afterRef };

  RefType refType;
  Integer<unsigned> integer;

  RelativeTimestamp(const char* name, RefType refType, unsigned maxDiff, unsigned scale = 1, unsigned nbits = 0)
      : refType(refType), integer(name, 0, maxDiff, scale, nbits)
  {
  }

  void write(Writer &w, unsigned timestamp, unsigned reference)
  {
    unsigned diff = (refType == beforeRef) ? reference - std::min(timestamp, reference)
                                           : std::max(timestamp, reference) - reference;
    integer.write(w, diff);
  }

  void read(Reader &r, unsigned &timestamp, unsigned reference) 
  {
    unsigned diff;
    integer.read(r, diff);
    timestamp = (refType == beforeRef) ? reference - diff : reference + diff;
  }

  bool checkQuantEqual(unsigned t1, unsigned ref1, unsigned t2, unsigned ref2)
  {
    unsigned diff1 = (refType == beforeRef) ? ref1 - std::min(t1, ref1) : std::max(t1, ref1) - ref1;
    unsigned diff2 = (refType == beforeRef) ? ref2 - std::min(t2, ref2) : std::max(t2, ref2) - ref2;

    return integer.checkQuantEqual(diff1, diff2); //(diff1 == diff2) || (diff1 == integer.maxVal && diff2 >= integer.maxVal);
  }
};


/**
 * TODO - not complete
 * 
 * a non-linear compressed relative timestamp. The resolution gets worse as the
 * time difference gets bigger
 */
// struct RelativeTimestampNonLinear
// {
//   enum RefType { beforeRef, afterRef };
// 
//   RelativeTimestampNonLinear(RefType refType, unsigned minStep, unsigned nbitsTotal = 8, unsigned nbitsValue = 4)
//       : integer(-maxDiff, maxDiff-1, scale, nbits)
//   {
//   }
// 
//   void write(Writer &w, unsigned timestamp, unsigned reference)
//   {
//     unsigned diff = (refType == beforeRef) ? (reference - timestamp) : (timestamp - reference);
//     diff /= minStep;
// 
//     const maxDiff = (1 << nBitsTotal) - 1;
//     if (diff > maxDiff)
//       diff = maxDiff;
// 
//     const unsigned valueMask = (1 << nbitsValue) - 1;
// 
//     // as long as the diff does not fit within the nBitsWithinScale, halve it
//     for (int iScale=0; (diff > valueMask) && (iScale < numScales); iScale++)
//       diff >>= 1;
// 
//     const unsigned scaleMask = (1 << nBitsScale) - 1;
//     unsigned value = ((iScale & scaleMask) << nBitsValue) | (diff & valueMask)
// 
//     w.writeBits(value, nBitsTotal)
//   }
// 
//   void read(Reader &r, unsigned &timestamp, unsigned reference) 
//   {
//     unsigned diff;
//     integer.read(r, diff);
//     timestamp = reference + diff;
//   }
// 
// private:
//   const RefType refType;
//   unsigned numScales;
//   unsigned nbitsWithinScale;
//   Integer<unsigned> integer;
// };



struct Bool
{
  const char* name;

  Bool(const char* name) : name(name) {}

  void write(Writer& w, bool value) { w.writeBits(value, 1, name); }
  void read(Reader &r, bool &value)
  {
    unsigned uval;
    r.readBits(uval, 1, name);
    value = uval;
  }
};

// struct Pose
// {
//   Pose(unsigned nbits)
// }


} // namespace BitStream


// encode/decode collections manually - simply write the length first as an integer