#ifndef TWIDDLER_H
#define TWIDDLER_H

namespace pipeline
{
  
  class Twiddler
  {
    //! Gets filled with (params, results) pairs when run() is called.
    std::map<Params, Params> results_;
    
    Twiddler() {}
    virtual ~Twiddle() {}
    
    //! Saves results to output_path and puts them in results_.
    //! output_path must not exist.
    void run(std::string output_path);
    
    virtual Params evaluate(const Params& params) = 0;
    //! Given the best current params, generate a new set of params.
    //! You can implement whatever search strategy you want here.
    //! Twiddler will check whether your new Params is a duplicate.
    virtual Params generateParamVariation(const Params& params) const = 0;
    //! By default, this checks to see if the double called "objective" has decreased from prev to curr;
    //! if so, it returns true.
    //! You can overload this to check, e.g., timing constraints.
    virtual bool isImprovement(const Params& prev, const Params& curr) const;

    static Params randomMerge(const Params& params0, const Params& params1);
    
  protected:
  };

}

#endif // TWIDDLER_H
