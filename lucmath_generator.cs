using System.Text;
using System.Xml.Linq;

string header =
@"// lucmath_gen.h

// MIT License
//
// Copyright (c) 2022 Robin Lind
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the ""Software""), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED ""AS IS"", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef LUCRAY_MATH_GEN_H
#define LUCRAY_MATH_GEN_H

#include <cmath>
#include <array>

namespace luc
{
";
string footer =
@"
}; // namespace luc

#endif /* LUCRAY_MATH_GEN_H */";
string base_vector =
@"template<typename T, size_t N>
union VectorTN
{
    VectorTN() : VectorTN(static_cast<T>(0)) {}
    VectorTN(const T& t) { std::fill(std::begin(E), std::end(E), t); }
    VectorTN(const std::array<T, N>& a) : E(a) {}
    std::array<T, N> E{};
};";

List<string> swizzle = new();
swizzle.Add("x,y,z,w");
swizzle.Add("r,g,b,a");
swizzle.Add("u,v,w");
swizzle.Add("w,h");
List<string> types = new();
types.Add("T");
types.Add("VectorTN<T,2>");
types.Add("VectorTN<T,3>");
types.Add("VectorTN<T,4>");
List<string> names = new();
names.Add("x");
names.Add("y");
names.Add("z");
names.Add("w");
List<string> usings = new();
usings.Add("Vector,float");
usings.Add("Double,double");
usings.Add("Int,int32_t");
usings.Add("Long,int64_t");
usings.Add("Bool,bool");
List<string> binary_ops = new();
binary_ops.Add("+,T");
binary_ops.Add("-,T");
binary_ops.Add("*,T");
binary_ops.Add("/,T");
binary_ops.Add("<,bool");
binary_ops.Add("<=,bool");
binary_ops.Add(">,bool");
binary_ops.Add(">=,bool");
binary_ops.Add("==,bool");
binary_ops.Add("!=,bool");
List<string> unary_math_ops = new();
unary_math_ops.Add("+=");
unary_math_ops.Add("-=");
unary_math_ops.Add("*=");
unary_math_ops.Add("/=");

List<string> first = new();
List<string> second = new();
List<string> third = new();
for (int i = 1; i < 4; i++)
{
    int N = i + 1;
    List<string> formats = new();
    foreach (var s in swizzle)
    {
        var split = s.Split(',');
        if (split.Length >= i+1)
        {
            string format = string.Join(',',split,0, i + 1);
            formats.Add(format);
        }
    }
    HashSet<string> c_hash = new()
    {
        "1","1,1,1,1"
    };
    for (int l = 0; l < i; l++)
    {
        var count = c_hash.Count;
        var c_list = c_hash.ToList();
        for (int k = 0; k < count; k++)
        {
            var c = c_list[k];
            for (int j = 1; j <= i; j++)
            {
                c_hash.Add(j + "," + c);
            }
        }
    }
    HashSet<string> c_hash_sum = new();
    foreach (var c in c_hash)
    {
        var values = c.Split(',').Select(int.Parse).ToArray();
        for (int l = 2; l < values.Length; l++)
        {
            var v_crop = values.Take(l).ToArray();
            var sum = v_crop.Sum();
            if (sum == 1 || sum == i + 1)
                c_hash_sum.Add(string.Join(',', v_crop));
        }
    }
    List<string> constructors = c_hash_sum.ToList();
    List<string> constructor_methods = new();
    {
        List<string> init = new();
        for (int j = 0; j < N; j++)
            init.Add("t");
        string single = $"\tVectorTN(const T& t) : E{{ {string.Join(", ", init)} }} {{}}";
        constructor_methods.Add(single);
    }
    foreach (var c in constructors)
    {
        int offset = 0;
        List<string> param = new();
        List<string> init = new();
        var values = c.Split(',').Select(int.Parse).ToArray();
        foreach (var v in values)
        {
            string name = string.Join(string.Empty, names.Skip(offset).Take(v));
            offset += v;
            param.Add($"const {types[v - 1]}& _{name}");
            if (v == 1)
            {
                init.Add($"_{name}");
            }
            else
            {
                for (int j = 0; j < v; j++)
                {
                    init.Add($"_{name}.{names[j]}");
                }
            }
        }
        string s = $"\tVectorTN({string.Join(", ", param)}) : E{{ {string.Join(", ", init)} }} {{}}";
        constructor_methods.Add(s);
    }
    List<string> unions = new();
    foreach (var swizz in swizzle)
    {
        var names_vars = swizz.Split(',');
        if (names_vars.Length >= N)
        {
            int names_used_count = -1;
            foreach (var c in constructors)
            {
                bool names_used = false;
                int offset = 0;
                List<string> vars = new();
                var values = c.Split(',').Select(int.Parse).ToArray();
                if (values.Contains(1))
                {
                    names_used = true;
                }
                foreach (var v in values)
                {
                    string name = string.Join(string.Empty, names_vars.Skip(offset).Take(v));
                    if (names_used && names_used_count > -1)
                    {
                        name += names_used_count;
                    }
                    offset += v;
                    vars.Add($"\t\t{types[v - 1]} {name};");
                }
                if (names_used)
                {
                    names_used_count++;
                }
                string s = $"\tstruct\n\t{{\n{string.Join("\n", vars)}\n\t}};";
                unions.Add(s);
            }
        }
    }
    StringBuilder vector_builder = new();
    StringBuilder op_builder = new();
    StringBuilder using_builder = new();
    string vector_head =
$"template<typename T>\nunion VectorTN<T, {N}>\n{"{"}\n\tVectorTN() : VectorTN(static_cast<T>(0)) {{}}\n";
    vector_builder.Append(vector_head);
    vector_builder.AppendLine(string.Join("\n", constructor_methods));
    vector_builder.Append(string.Join("\n", unions));
    vector_builder.Append($"\n\tstd::array<T, {N}> E{{}};\n}};");
    foreach (var o in binary_ops)
    {
        var split = o.Split(',');
        var op = split[0];
        var rt = split[1];
        var signature_vv = $"template<typename T>\nauto operator{op}(const VectorTN<T, {N}>& t, const VectorTN<T, {N}>& u)";
        var signature_vs = $"template<typename T>\nauto operator{op}(const VectorTN<T, {N}>& t, const T& u)";
        var signature_sv = $"template<typename T>\nauto operator{op}(const T& t, const VectorTN<T, {N}>& u)";

        string gen_op_func(bool t, bool u)
        {
            List<string> ops = new();
            for (int j = 0; j < N; j++)
            {
                var variable = $".{names[j]}";
                var tV = t ? variable : "";
                var uV = u ? variable : "";
                ops.Add($"t{tV} {op} u{uV}");
            }
            return $"\n{"{"}\n\tVectorTN<{rt}, {N}> result({string.Join(", ", ops)});\n\treturn result;\n}}";
        }
        op_builder.Append($"\n{signature_vv}{gen_op_func(true, true)}");
        op_builder.Append($"\n{signature_vs}{gen_op_func(true, false)}");
        op_builder.Append($"\n{signature_sv}{gen_op_func(false, true)}");
    }
    foreach (var op in unary_math_ops)
    {
        var signature_vv = $"template<typename T>\nauto operator{op}(const VectorTN<T, {N}>& t, const VectorTN<T, {N}>& u)";
        var signature_vs = $"template<typename T>\nauto operator{op}(const VectorTN<T, {N}>& t, const T& u)";

        op_builder.Append($"\n{signature_vv}\n{"{"}\n\tt = t + u;\n}}");
        op_builder.Append($"\n{signature_vs}\n{"{"}\n\tt = t + u;\n}}");
    }
    {
        string gen_op_func()
        {
            List<string> ops = new();
            for (int j = 0; j < N; j++)
            {
                ops.Add($"-t.{names[j]}");
            }
            return $"\n{"{"}\n\tVectorTN<T, {N}> result({string.Join(", ", ops)});\n\treturn result;\n}}";
        }
        var signature_unary_negate = $"template<typename T>\nauto operator-(const VectorTN<T, {N}>& t)";
        op_builder.Append($"\n{signature_unary_negate}{gen_op_func()}");
    }
    foreach (var u in usings)
    {
        var split = u.Split(',');
        var us = $"\nusing {split[0]}{i + 1} = VectorTN<{split[1]},{i + 1}>;";
        using_builder.Append(us);
    }
    first.Add(vector_builder.ToString());
    second.Add(op_builder.ToString());
    third.Add(using_builder.ToString());
}
StringBuilder final_builder = new();
final_builder.AppendLine(header);
final_builder.AppendLine(base_vector);
final_builder.AppendLine(string.Join("\n\n", first));
final_builder.AppendLine(string.Join("", second));
final_builder.AppendLine(string.Join("\n", third));
final_builder.AppendLine(footer);
File.WriteAllText("\\\\wsl$\\Ubuntu-22.04\\home\\lorto6\\lucray\\lucmath_gen.h", final_builder.ToString());