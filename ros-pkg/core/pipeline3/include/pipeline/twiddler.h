#ifndef TWIDDLER_H
#define TWIDDLER_H

#include <signal.h>
#include <bag_of_tricks/bag_of_tricks.h>

namespace pipeline
{
  
  class Twiddler
  {
    //! User-defined results of evaluating a Params set.
    typedef Dictionary<std::string, double> Results;

    std::map<Params, Results> results_;
    
    Twiddler();
    virtual ~Twiddle() {}
    //! path must not exist and will be created.
    void initialize(std::string path);
    //! path must exist.  Loads existing results.
    void load(std::string path);
    //! Fills results_ and saves to path_ as new evaluations are made.
    void run(const Params& init);
    //! results_ must be non-empty.  Picks up where it left off.
    void resume(); 

    /************************************************************
     * Functions to be implemented
     ************************************************************/

    //! By default the returned Results should contain an "objective" field.
    //! By overloading this method you can interpret additional results.
    //! For example, you might include "time" in addition to "objective", then overload isImprovement to
    //! optimize the objective as long as timing constraints are met.
    //! path is the directory to which you can save things if you want.
    virtual Results evaluate(const Params& params, std::string path) = 0;
    //! Given the best current params, generate a new set of params.
    //! You can implement whatever search strategy you want here.
    //! Twiddler will check whether your new Params is a duplicate.
    virtual Params generateParamVariation(const Params& params) const = 0;
    //! By default, this returns the field called "objective".
    //! You can overload this to handle constraints as well as different objectives.
    //! Lower is better.
    virtual double objective(const Results& results) const;
    
    /************************************************************
     * Useful functions for generating parameter variations
     ************************************************************/
    
    static Params randomMerge(const Params& params0, const Params& params1);
    
  protected:
    std::string path_;
    size_t next_id_;
  };

}

#endif // TWIDDLER_H
