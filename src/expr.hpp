// copyright 2026 pencil authors
// SPDX-License-Identifier: Apache-2.0
// licensed under the apache license, version 2.0 (the "license");
// you may not use this file except in compliance with the license.
// you may obtain a copy of the license at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// unless required by applicable law or agreed to in writing, software
// distributed under the license is distributed on an "as is" basis,
// without warranties or conditions of any kind, either express or implied.
// see the license for the specific language governing permissions and
// limitations under the license.

// expr.hpp - Tiny math expression evaluator for Pencil
#pragma once

#include <cmath>
#include <cctype>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace pencil {

class Expr {
public:
    virtual ~Expr() = default;
    virtual float eval(float x, float y, float z) const = 0;
};

// ---- AST node types -------------------------------------------------------

struct NumExpr : Expr {
    float v = 0.0f;
    explicit NumExpr(float v_) : v(v_) {}
    float eval(float, float, float) const override { return v; }
};

struct VarExpr : Expr {
    int idx = 0; // 0=x, 1=y, 2=z
    explicit VarExpr(int i) : idx(i) {}
    float eval(float x, float y, float z) const override {
        return (idx == 0) ? x : (idx == 1) ? y : z;
    }
};

enum class BinOp { Add, Sub, Mul, Div, Pow };

struct BinExpr : Expr {
    BinOp op;
    std::shared_ptr<Expr> lhs, rhs;
    BinExpr(BinOp o, std::shared_ptr<Expr> l, std::shared_ptr<Expr> r)
        : op(o), lhs(std::move(l)), rhs(std::move(r)) {}
    float eval(float x, float y, float z) const override {
        float a = lhs->eval(x, y, z);
        float b = rhs->eval(x, y, z);
        switch (op) {
            case BinOp::Add: return a + b;
            case BinOp::Sub: return a - b;
            case BinOp::Mul: return a * b;
            case BinOp::Div: return (b != 0.0f) ? a / b : 0.0f;
            case BinOp::Pow: return std::pow(a, b);
        }
        return 0.0f;
    }
};

struct NegExpr : Expr {
    std::shared_ptr<Expr> inner;
    explicit NegExpr(std::shared_ptr<Expr> i) : inner(std::move(i)) {}
    float eval(float x, float y, float z) const override {
        return -inner->eval(x, y, z);
    }
};

struct CallExpr : Expr {
    std::string name;
    std::vector<std::shared_ptr<Expr>> args;
    CallExpr(std::string n, std::vector<std::shared_ptr<Expr>> a)
        : name(std::move(n)), args(std::move(a)) {}
    float eval(float x, float y, float z) const override {
        if (args.empty()) return 0.0f;
        float a = args[0]->eval(x, y, z);
        if (name == "sin")  return std::sin(a);
        if (name == "cos")  return std::cos(a);
        if (name == "tan")  return std::tan(a);
        if (name == "sqrt") return (a >= 0.0f) ? std::sqrt(a) : 0.0f;
        if (name == "abs")  return std::fabs(a);
        if (name == "exp")  return std::exp(a);
        if (name == "log")  return (a > 0.0f) ? std::log(a) : 0.0f;
        if (args.size() >= 2) {
            float b = args[1]->eval(x, y, z);
            if (name == "min") return std::fmin(a, b);
            if (name == "max") return std::fmax(a, b);
            if (name == "pow") return std::pow(a, b);
        }
        return 0.0f;
    }
};

// ---- Recursive descent parser --------------------------------------------

class ExprParser {
public:
    ExprParser(const std::string& src) : src_(src), pos_(0) {}

    std::shared_ptr<Expr> parse(std::string& err) {
        skip_ws();
        auto e = parseExpr(err);
        if (!e) return nullptr;
        skip_ws();
        if (pos_ < src_.size()) {
            err = "unexpected character '" + std::string(1, src_[pos_]) +
                  "' at position " + std::to_string(pos_);
            return nullptr;
        }
        return e;
    }

private:
    const std::string& src_;
    size_t pos_;

    char peek() const { return (pos_ < src_.size()) ? src_[pos_] : '\0'; }
    char get()        { return (pos_ < src_.size()) ? src_[pos_++] : '\0'; }
    void skip_ws() {
        while (pos_ < src_.size() && std::isspace((unsigned char)src_[pos_])) pos_++;
    }

    std::shared_ptr<Expr> parseExpr(std::string& err) {
        auto lhs = parseTerm(err);
        if (!lhs) return nullptr;
        while (true) {
            skip_ws();
            char c = peek();
            if (c == '+') { get(); auto r = parseTerm(err); if (!r) return nullptr; lhs = std::make_shared<BinExpr>(BinOp::Add, lhs, r); }
            else if (c == '-') { get(); auto r = parseTerm(err); if (!r) return nullptr; lhs = std::make_shared<BinExpr>(BinOp::Sub, lhs, r); }
            else break;
        }
        return lhs;
    }

    std::shared_ptr<Expr> parseTerm(std::string& err) {
        auto lhs = parseFactor(err);
        if (!lhs) return nullptr;
        while (true) {
            skip_ws();
            char c = peek();
            if (c == '*') { get(); auto r = parseFactor(err); if (!r) return nullptr; lhs = std::make_shared<BinExpr>(BinOp::Mul, lhs, r); }
            else if (c == '/') { get(); auto r = parseFactor(err); if (!r) return nullptr; lhs = std::make_shared<BinExpr>(BinOp::Div, lhs, r); }
            else break;
        }
        return lhs;
    }

    std::shared_ptr<Expr> parseFactor(std::string& err) {
        auto base = parseUnary(err);
        if (!base) return nullptr;
        skip_ws();
        if (peek() == '^') {
            get();
            auto exp = parseFactor(err); // right-assoc
            if (!exp) return nullptr;
            return std::make_shared<BinExpr>(BinOp::Pow, base, exp);
        }
        return base;
    }

    std::shared_ptr<Expr> parseUnary(std::string& err) {
        skip_ws();
        if (peek() == '-') {
            get();
            auto inner = parseUnary(err);
            if (!inner) return nullptr;
            return std::make_shared<NegExpr>(inner);
        }
        return parsePrimary(err);
    }

    std::shared_ptr<Expr> parsePrimary(std::string& err) {
        skip_ws();
        char c = peek();
        if (c == '\0') { err = "unexpected end of expression"; return nullptr; }

        if (c == '(') {
            get();
            auto e = parseExpr(err);
            if (!e) return nullptr;
            skip_ws();
            if (peek() != ')') { err = "expected ')'"; return nullptr; }
            get();
            return e;
        }

        if (std::isdigit((unsigned char)c) || c == '.') {
            return parseNumber(err);
        }

        if (std::isalpha((unsigned char)c) || c == '_') {
            return parseIdent(err);
        }

        err = std::string("unexpected character '") + c + "'";
        return nullptr;
    }

    std::shared_ptr<Expr> parseNumber(std::string& err) {
        size_t start = pos_;
        bool seenDot = false;
        while (pos_ < src_.size() &&
               (std::isdigit((unsigned char)src_[pos_]) ||
                src_[pos_] == '.')) {
            if (src_[pos_] == '.') {
                if (seenDot) break;
                seenDot = true;
            }
            pos_++;
        }
        // optional scientific notation: e.g
        if (pos_ < src_.size() && (src_[pos_] == 'e' || src_[pos_] == 'E')) {
            pos_++;
            if (pos_ < src_.size() && (src_[pos_] == '+' || src_[pos_] == '-')) pos_++;
            while (pos_ < src_.size() && std::isdigit((unsigned char)src_[pos_])) pos_++;
        }
        std::string numStr = src_.substr(start, pos_ - start);
        try {
            return std::make_shared<NumExpr>(std::stof(numStr));
        } catch (...) {
            err = "invalid number: " + numStr;
            return nullptr;
        }
    }

    std::shared_ptr<Expr> parseIdent(std::string& err) {
        size_t start = pos_;
        while (pos_ < src_.size() &&
               (std::isalnum((unsigned char)src_[pos_]) || src_[pos_] == '_')) {
            pos_++;
        }
        std::string name = src_.substr(start, pos_ - start);
        skip_ws();

        // function call?
        if (peek() == '(') {
            get();
            std::vector<std::shared_ptr<Expr>> args;
            skip_ws();
            if (peek() != ')') {
                auto a = parseExpr(err);
                if (!a) return nullptr;
                args.push_back(a);
                skip_ws();
                while (peek() == ',') {
                    get();
                    auto next = parseExpr(err);
                    if (!next) return nullptr;
                    args.push_back(next);
                    skip_ws();
                }
            }
            if (peek() != ')') { err = "expected ')'"; return nullptr; }
            get();
            return std::make_shared<CallExpr>(name, std::move(args));
        }

        // variable
        if (name == "x") return std::make_shared<VarExpr>(0);
        if (name == "y") return std::make_shared<VarExpr>(1);
        if (name == "z") return std::make_shared<VarExpr>(2);
        // allow constants pi, e as built-ins
        if (name == "pi") return std::make_shared<NumExpr>(3.14159265358979323846f);
        if (name == "e")  return std::make_shared<NumExpr>(2.71828182845904523536f);

        err = "unknown identifier: " + name;
        return nullptr;
    }
};

// public entry: parse a string into an Expr
inline std::shared_ptr<Expr> parse_expression(const std::string& src,
                                              std::string& err_out) {
    ExprParser p(src);
    return p.parse(err_out);
}

} // namespace pencil
