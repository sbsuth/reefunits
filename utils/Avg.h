#ifndef AVG_H
#define AVG_H

template <int N, typename T=short>
class Avg {
  public:
    Avg( T center=0 ) 
       : m_avg(center), m_nVals(0)
    {
       for ( int i=0; i < N; i++ )
         m_vals[i] = center;
    }

    bool isFull() {
        return (m_nVals >= N);
    }
    T update( T newVal ) {
        T sum = newVal;
        for ( int i=0; i < (N-1); i++ ) {
          bool good = (i > ((N-2)-m_nVals));
          m_vals[i] = (good ? m_vals[i+1] : newVal);
          sum += m_vals[i];
        }
        m_vals[N-1] = newVal;
        m_avg = sum/T(N);
        if (m_nVals < N)
            m_nVals++;
        return avg();
    }
    T avg() const {
        return m_avg;
    }
    unsigned char m_nVals;
    T m_avg;
    T m_vals[N];
};

template <int N, typename T>
class AvgThresh : public Avg<N,T> {
  public:
    AvgThresh( T center, T lowThresh, T highThresh )
      : Avg<N,T>( center ), m_lowThresh(lowThresh), m_highThresh(highThresh) {

        // Set limits at 8X the distance from center.
        #define LIM_MUL 8
        m_lowLimit = center - ((center - lowThresh)*LIM_MUL);
        m_highLimit = center + ((highThresh - center)*LIM_MUL);
    }
    bool isLow() {
      return (Avg<N,T>::m_avg < m_lowThresh);
    }
    bool isHigh() {
      return (Avg<N,T>::m_avg > m_highThresh);
    }
    bool isInRange() {
      return (!isLow() && !isHigh());
    }
    T update( T newVal ) {
        // Ignore anything outside limits
        if ((newVal < m_lowLimit) || (newVal > m_highLimit))
            return Avg<N,T>::avg();
        else
            return Avg<N,T>::update(newVal);
    }
    T m_lowLimit;
    T m_highLimit;
    T m_lowThresh;
    T m_highThresh;
};


#endif
