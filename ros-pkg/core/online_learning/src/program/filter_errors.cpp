#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <online_learning/grid_classifier.h>

using namespace std;

void explode(TrackDataset* td_ptr)
{
  TrackDataset& td = *td_ptr;
  cout << "Before exploding: " << endl;
  cout << td.status("  ", false);
  for(size_t i = 0; i < td.size(); ++i) {
    while(td[i].size() > 1) {
      Dataset::Ptr tr(new Dataset);
      tr->applyNameMappings(td);
      tr->instances_.push_back(td[i].instances_.back());
      td[i].instances_.pop_back();
      td.tracks_.push_back(tr);
    }
  }
  cout << "After exploding: " << endl;
  cout << td.status("  ", false);
}

void process(const GridClassifier& gc, const TrackDataset& td,
             map<string, TrackDataset::Ptr>* fps,
             map<string, TrackDataset::Ptr>* fns)
{
  NameMapping cmap = gc.nameMapping("cmap");
  vector<Label> predictions(td.size());
  vector<Label> annotations(td.size());

  cout << "Classifying..." << endl;

  // Use a random order because many tds are sorted by size
  // and that leads to terrible core utilization.
  vector<size_t> index(td.size());
  for(size_t i = 0; i < index.size(); ++i)
    index[i] = i;
  random_shuffle(index.begin(), index.end());
  
  #pragma omp parallel for
  for(size_t i = 0; i < index.size(); ++i) {
    size_t idx = index[i];
    annotations[idx] = td.label(idx);
    predictions[idx] = gc.classifyTrack(td[idx]);
  }

  cout << "Copying errors..." << endl;
  for(size_t i = 0; i < td.size(); ++i) {
    const Label& annotation = annotations[i];
    const Label& prediction = predictions[i];
    for(size_t c = 0; c < cmap.size(); ++c) {
      if(prediction(c) > 0 && annotation(c) < 0) {
        (*fps)[cmap.toName(c)]->tracks_.push_back(td.tracks_[i]);
        (*fps)[cmap.toName(c)]->tracks_.back()->setLabel(prediction);
      }
      else if(prediction(c) < 0 && annotation(c) > 0) {
        (*fns)[cmap.toName(c)]->tracks_.push_back(td.tracks_[i]);
        (*fns)[cmap.toName(c)]->tracks_.back()->setLabel(prediction);
      }
    }
  }
}

void save(const string& dir, const string& type, const map<string, TrackDataset::Ptr>& datasets)
{
  map<string, TrackDataset::Ptr>::const_iterator it;
  for(it = datasets.begin(); it != datasets.end(); ++it) {
    ostringstream oss;
    oss << dir << "/" << it->first << "_" << type << ".td";
    cout << "Saving to " << oss.str() << "..." << endl;
    it->second->save(oss.str());
  }
}

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string classifier_path;
  string output_dir;
  vector<string> dataset_paths;
  vector<string> class_names;
  
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("classifier,c", bpo::value<string>(&classifier_path), "")
    ("output,o", bpo::value<string>(&output_dir), "")
    ("datasets,d", bpo::value< vector<string> >(&dataset_paths)->required()->multitoken(), "")
    ("all-frames,f", "Filter out all frame classification errors rather than just track classification errors.")
    ("class-names", bpo::value(&class_names)->required()->multitoken(), "")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] -d TD [ TD ... ] -c CLASSIFIER -o OUTPUT_DIR" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  if(!bfs::exists(output_dir))
    bfs::create_directory(output_dir);
  
  // -- Load the classifier.
  GridClassifier gc;
  cout << "Loading classifier " << classifier_path << "." << endl;
  gc.load(classifier_path);

  // -- Set up the class map to use. 
  NameMapping cmap;
  cmap.addNames(class_names);
  cout << "Using cmap: " << endl;
  cout << cmap.status("  ") << endl;
  gc.applyNameMapping("cmap", cmap);
  
  // -- Set up storage for FPs and FNs of each class.
  map<string, TrackDataset::Ptr> fps;
  map<string, TrackDataset::Ptr> fns;
  for(size_t c = 0; c < cmap.size(); ++c) {
    fps[cmap.toName(c)] = TrackDataset::Ptr(new TrackDataset);
    fps[cmap.toName(c)]->applyNameMappings(gc);
    fns[cmap.toName(c)] = TrackDataset::Ptr(new TrackDataset);
    fns[cmap.toName(c)]->applyNameMappings(gc);
  }

  // -- Find FPs and FNs.
  for(size_t i = 0; i < dataset_paths.size(); ++i) {
    TrackDataset td;
    cout << "Loading dataset " << dataset_paths[i] << "." << endl;
    td.load(dataset_paths[i]);
    td.applyNameMapping("cmap", cmap);
    ROS_ASSERT(gc.nameMappingsAreEqual(td));
    if(opts.count("all-frames"))
      explode(&td);
    process(gc, td, &fps, &fns);
  }

  // -- Save FPs and FNs.
  save(output_dir, "fps", fps);
  save(output_dir, "fns", fns);
  cout << "Done." << endl;
    
  return 0;
}
