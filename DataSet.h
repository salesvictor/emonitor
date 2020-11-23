//#ifndef DATASET_UTILS_H
//#define DATASET_UTILS_H

template <int max_size>
struct Measurements
{
  uint32_t first_epoch;
  float    current[max_size];
  float    power[max_size];
};

struct Measure
{
  float current;
  float power;
};

template <int max_size>
class DataSet
{
  public:
    void setFirstEpoch(int first_epoch) {
      measurements_.first_epoch = first_epoch;
      n_measures_ = 0; // Goes back to beggining
    }

    // NOTE: Make sure to not overflow!!
    void addData(Measure data)
    {
      measurements_.current[n_measures_] = data.current;
      measurements_.power[n_measures_]   = data.power;
      ++n_measures_;
    }

    const int getSize() const
    {
      return n_measures_;
    }

    const Measurements<max_size> &getData() const
    {
      return measurements_;
    }

    const Measure getData(int n_pos) const {
      return {measurements_.current[n_pos], measurements_.power[n_pos]};
    }
    
  private:
    int                    n_measures_ = 0;
    Measurements<max_size> measurements_;   // Acts as a circular array
};
