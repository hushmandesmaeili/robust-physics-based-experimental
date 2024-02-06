import math
import sympy as sp

class MathUtils:
    # @classmethod
    # def taylor_approximation(cls, func, order):
    #     def polynomial(x):
    #         result = 0
    #         for n in range(order + 1):
    #             result += func(n) * (x ** n) / math.factorial(n)
    #         return result
    #     return polynomial

    @staticmethod
    def taylor_approximation(func, degree, point):
        t = sp.symbols('t')
        taylor = sp.series(func, t, point, n=degree+1).removeO()
        return taylor