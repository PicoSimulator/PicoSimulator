#pragma once



template<class T>
template<class U, size_t N>
T* array_converter<T>::to_array(U (&a)[N]){
  return reinterpret_cast<T*>(a);
}