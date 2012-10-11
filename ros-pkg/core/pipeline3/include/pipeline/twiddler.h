#ifndef TWIDDLER_H
#define TWIDDLER_H

#include <signal.h>
#include <boost/filesystem.hpp>
#include <ros/assert.h>
#include <ros/console.h>
#include <bag_of_tricks/bag_of_tricks.h>
#include <pipeline/params.h>

namespace pipeline
{
  
  class Twiddler
  {
  public:
    //! User-defined results of evaluating a Params set.
    typedef Dictionary<std::string, double> Results;
    std::map<Params, Results> results_;
    
    Twiddler();
    virtual ~Twiddler() {}
    //! path must not exist and will be created.
    //! Fills results_ and saves to path_ as new evaluations are made.
    void run(const Params& init, std::string rootpath);
    //! path must exist.  Loads existing results.
    //! results_ must be non-empty.  Finds the best set of results and picks up where it left off.
    void resume(std::string rootpath); 

    
    /************************************************************
     * Functions to be implemented
     ************************************************************/

    //! You can place additional output in evalpath, if needed.
    virtual Results evaluate(const Params& params, std::string evalpath) = 0;
    //! Given the best current params, generate a new set of params.
    //! You can implement whatever search strategy you want here.
    //! Twiddler will check whether your new Params is a duplicate.
    virtual Params generateParamVariation(Params params) const = 0;
    //! By default, this returns the field called "objective".
    //! You can overload this to handle constraints as well as different objectives.
    //! Lower is better.
    virtual double objective(const Results& results) const;
    //! This is called any time an improved set of params is found.
    virtual void improvementHook(const Params& params, const Results& results, std::string evalpath) const;
    //! Termination criteria.  Never terminates by default.
    virtual bool done(const Results& results) const;

    
    /************************************************************
     * Miscellaneous things.
     ************************************************************/

    void getBest(Params* best_params, Results* best_results) const;

    
    /************************************************************
     * Useful functions for generating parameter variations
     ************************************************************/
    
    static Params randomMerge(const Params& params0, const Params& params1);
    
  protected:
    std::string rootpath_;
    size_t next_id_;

    void twiddle();
  };

}

#endif // TWIDDLER_H
