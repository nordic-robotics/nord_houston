{-# LANGUAGE OverloadedStrings #-}
module SeqSel.Printer (printExpr, printVar, printFunc, printTree) where

import Data.Monoid
import qualified Data.Text.Lazy as L
import SeqSel.Parser

indented :: Int -> L.Text -> L.Text
indented = (<>) . ("\n" <>) . L.pack . flip replicate ' '

printExpr :: Int -> Expr -> L.Text
printExpr n (Selector node children) = indented n ("/* " <> L.pack node <> " */")
                                    <> indented n ("(   " <> children')
                                    <> indented n ")"
  where
    children' = L.intercalate (indented n "|| ") (map (printExpr (n + 4)) children)

printExpr n (Sequence node children) = indented n ("/* " <> L.pack node <> " */")
                                    <> indented n ("(" <> children' <> ")")
  where
    children' = L.intercalate " && " (map (printExpr (n + 4)) children)

printExpr _ (Condition cond) = L.pack cond

printExpr _ (Call fun) = L.pack fun

printVar :: Int -> Var -> L.Text
printVar n (Var str) = indented n (L.pack str <> ";")

printFunc :: Int -> Expr -> L.Text
printFunc n expr = indented n "bool run()"
                <> indented n "{"
                <> indented (n + 4) ("return" <> printExpr (n + 4) expr <> ";")
                <> indented n "}"

printTree :: [Var] -> Expr -> L.Text
printTree vars expr = "#pragma once\n\nclass " <> L.pack (name expr)
                   <> "\n{\npublic:" <> vars'
                   <> "\n" <> printFunc 4 expr <> "\n};"
  where
    vars' = L.concat $ map (printVar 4) vars
