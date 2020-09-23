#pragma once

#include <algorithm>
#include <random>
#include <iostream>

// vector
template <typename T>
static bool inArray(T a, std::vector<T> &arr) {
  auto itr = std::find(arr.begin(), arr.end(), a);
  return itr != arr.end();
}

template <typename T>
static bool inArray(std::vector<T> &sArr, std::vector<T> &lArr) {
  return std::all_of(sArr.begin(), lArr.end(),
                     [lArr](T ele) {
                       auto itr = std::find(lArr.begin, lArr.end(), ele);
                       return itr != lArr.end(); });
}

template <typename T>
static void openToClose(T a, std::vector<T>& OPEN, std::vector<T> &CLOSE) {
  auto itr = std::find(OPEN.begin(), OPEN.end(), a);
  if (itr != OPEN.end()) {
    CLOSE.push_back(*itr);
    OPEN.erase(itr);
  }
}

template <typename T>
static T randomChoose(std::vector<T> &arr, std::mt19937* MT) {
  std::uniform_int_distribution<int> chooser(0, arr.size() - 1);
  return arr[chooser(*MT)];
}

template <typename T>
static T printVector(std::vector<T> &arr) {
  for (auto ele : arr) std::cout << ele << "    ";
  std::cout << "\n";
}
