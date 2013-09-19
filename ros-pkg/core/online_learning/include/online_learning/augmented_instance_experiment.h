#include <serializable/serializable.h>

class Bare : public Serializable
{
  void serialize(std::ostream& out) const {}
  void deserialize(std::istream& in) {}
};

class ExtraCustomData : public Serializable
{
public:
  double val_;
  
  void serialize(std::ostream& out) const
  {
    out.write((char*)&val_, sizeof(val_));
  }

  void deserialize(std::istream& in)
  {
    in.read((char*)&val_, sizeof(val_));
  }
};

template<typename T>
class Instance : public Serializable
{
public:
  int val_;
  T raw_;
  
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
  void something();
};

template<typename T>
void Instance<T>::serialize(std::ostream& out) const
{
  out.write((char*)&val_, sizeof(val_));
  out << raw_;
}

template<typename T>
void Instance<T>::deserialize(std::istream& in)
{
  in.read((char*)&val_, sizeof(val_));
  in >> raw_;
}

template<typename T>
void Instance<T>::something()
{
  std::cout << "Hi." << std::endl;
}
