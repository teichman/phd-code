#include <asp/asp.h>

using namespace std;

void ASPWeights::serialize(std::ostream& out) const
{
  serializeNameMappings(out);
  eigen_extensions::serialize(nweights_, out);
  eigen_extensions::serialize(eweights_, out);
}

void ASPWeights::deserialize(std::istream& in)
{
  deserializeNameMappings(in);
  eigen_extensions::deserialize(in, &nweights_);
  eigen_extensions::deserialize(in, &eweights_);
}

void ASPWeights::_applyNameTranslator(const std::string& id, const NameTranslator2& translator)
{
  ROS_ASSERT(id == "nmap" || id == "emap");
  if(id == "nmap")
    translator.translate(&nweights_, 0.0);
  else if(id == "emap")
    translator.translate(&eweights_, 0.0);
}

std::string ASPWeights::status(const std::string& prefix) const
{
  ostringstream oss;
  ROS_ASSERT((size_t)nweights_.rows() == nameMapping("nmap").size());
  ROS_ASSERT((size_t)eweights_.rows() == nameMapping("emap").size());

  oss << prefix << "Node potentials" << endl;
  for(size_t i = 0; i < nameMapping("nmap").size(); ++i) 
    oss << prefix << "  " << setiosflags(ios::left) << setw(25) << nameMapping("nmap").toName(i) << ": " << nweights_[i] << endl;

  oss << prefix << "Edge potentials" << endl;
  for(size_t i = 0; i < nameMapping("emap").size(); ++i) 
    oss << prefix << "  " << setiosflags(ios::left) << setw(25) << nameMapping("emap").toName(i) << ": " << eweights_[i] << endl;

  return oss.str();
}

