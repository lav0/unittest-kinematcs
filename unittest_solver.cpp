#include "gtest\gtest.h"

#include "..\NelderMeadSolver\VariableSetGeneric.h"


using namespace nmsolver;

TEST(Solver, VariableSetGenetic)
{
  using VariableSetDouble = VariableSetGeneric<ValueWrapperDouble>;

  size_t size = 5;
  auto var = std::make_unique<VariableSetDouble>(size);
  for (size_t i = 0; i < size; ++i)
  {
    var->set_limits(i, -45, 45);
    var->set_var(i, (i+1) * 10.0);
  }
  auto var_copy = var->deep_copy();

  EXPECT_EQ(var->size(), var_copy->size());
  EXPECT_EQ(size, var_copy->size());

  for (size_t i = 0; i < size; ++i)
  {
    auto v = var->get_var(i);
    auto w = var_copy->get_var(i);

    EXPECT_EQ(v, w);
  }

  auto doubled = var->multiply(2.0);
  auto expected = { 20.0, 40.0, 45.0, 45.0, 45.0 };
  auto exp = expected.begin();

  for (size_t i = 0; i < size; ++i)
  {
    auto w = doubled->get_var(i);

    EXPECT_EQ(*exp, w);

    ++exp;
  }

  auto subst = doubled->subtract(var.get());
  auto expected0 = { 10.0, 20.0, 15.0, 5.0, 0.0 };
  auto exp0 = expected0.begin();

  for (size_t i = 0; i < size; ++i)
  {
    auto w = subst->get_var(i);

    EXPECT_EQ(*exp0, w);

    ++exp0;
  }
  EXPECT_TRUE(true);
}