#ifndef AVG_H
#define AVG_H

template <int N>
class Avg {
  public:
    Avg( short center ) 
       : m_avg(center)
    {
       for ( int i=0; i < N; i++ )
         m_vals[i] = center;
    }

    short update( short newVal ) {
        int sum = newVal;
        m_vals[N-1] = sum;
        for ( int i=0; i < (N-1); i++ ) {
          m_vals[i] = m_vals[i+1];
          sum += m_vals[i];
        }
        m_avg = sum/N;
        return avg();
    }
    short avg() const {
        return m_avg;
    }
    short m_avg;
    short m_vals[N];
};

template <int N>
class AvgThresh : public Avg<N> {
  public:
    AvgThresh( short center, short lowThresh, short highThresh )
      : Avg<N>( center ), m_lowThresh(lowThresh), m_highThresh(highThresh) {
    }
    bool isLow() {
      return (Avg<N>::m_avg < m_lowThresh);
    }
    bool isHigh() {
      return (Avg<N>::m_avg > m_highThresh);
    }
    bool isInRange() {
      return (!isLow() && !isHigh());
    }
    short m_lowThresh;
    short m_highThresh;
};


#endif
