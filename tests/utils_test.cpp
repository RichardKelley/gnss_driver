#include "gtest/gtest.h"

#include "gnss/utils.h"

namespace gnss_driver {

  TEST(utils, encode_base64) {
    std::string str64;
    
    std::string str("The quick brown fox jumps over the lazy dog..");
    EXPECT_STREQ("VGhlIHF1aWNrIGJyb3duIGZveCBqdW1wcyBvdmVyIHRoZSBsYXp5IGRvZy4u",
                 encode_base64(str).c_str());
    
    str.resize(str.size() - 1);
    EXPECT_STREQ("VGhlIHF1aWNrIGJyb3duIGZveCBqdW1wcyBvdmVyIHRoZSBsYXp5IGRvZy4=",
                 encode_base64(str).c_str());
    
    str.resize(str.size() - 1);
    EXPECT_STREQ("VGhlIHF1aWNrIGJyb3duIGZveCBqdW1wcyBvdmVyIHRoZSBsYXp5IGRvZw==",
                 encode_base64(str).c_str());
  }

}  // namespace gnss_driver


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
