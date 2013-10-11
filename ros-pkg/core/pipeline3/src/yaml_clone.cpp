#include <iostream>
#include <yaml-cpp/yaml.h>

using namespace std;

int main(int argc, char** argv)
{
  {
    YAML::Node one;
    YAML::Node two;

    assert(!one["thisdoesnotexist"]);
    cout << "thisdoesnotexist: " << one["thisdoesnotexist"] << endl;
    
    // -- clone.  Changing two should not affect one.
    one["Foo"] = "Bar";
    two = YAML::Clone(one);
    two["Foo"] = "Baz";
    cout << one["Foo"] << " " << two["Foo"] << endl;
    assert(!one.is(two) && !two.is(one));
    assert(one["Foo"].as<string>() != two["Foo"].as<string>());
  }

  {
    YAML::Node one;
    YAML::Node two;

    // -- operator=.  Changing two should affect one.
    one["Foo"] = "Bar";
    two = one;
    two["Foo"] = "Baz";
    cout << one["Foo"] << " " << two["Foo"] << endl;
    assert(one.is(two) && two.is(one));
    assert(one["Foo"].as<string>() == two["Foo"].as<string>());

    // -- clone.  Changing two should not affect one.
    one["Foo"] = "Bar";
    two = YAML::Clone(one);
    two["Foo"] = "Baz";
    // ... but it does.
    cout << one["Foo"] << " " << two["Foo"] << endl;
    assert(!one.is(two) && !two.is(one));
    assert(one["Foo"].as<string>() != two["Foo"].as<string>());
  }
  
  return 0;
}
