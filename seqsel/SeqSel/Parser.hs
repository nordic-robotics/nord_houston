module SeqSel.Parser (Var(..), Expr(..), parseFile, name) where

import Prelude hiding (sequence)
import Text.Parsec
import Text.Parsec.String
import Data.String.Utils
import Control.Monad hiding (sequence)

data Var = Var String
            deriving (Eq, Show)

data Expr = Selector String [Expr]
          | Sequence String [Expr]
          | Condition String
          | Call String
            deriving (Eq, Show)

type Defines = [(String, String)]

traverse' :: (String -> String) -> Expr -> Expr
traverse' f (Selector n children) = Selector n (map (traverse' f) children)
traverse' f (Sequence n children) = Sequence n (map (traverse' f) children)
traverse' f (Condition str) = Condition (f str)
traverse' f (Call str) = Call (f str)

name :: Expr -> String
name (Selector n _) = n
name (Sequence n _) = n
name _ = ""

spaces1 :: Parser ()
spaces1 = many1 (char ' ') *> pure ()

atom :: Parser String
atom = (:) <$> lower <*> many (alphaNum <|> oneOf "_")

rest :: Parser String
rest = do
    str <- many (noneOf "\n")
    skipMany newline
    return (str)

replace' :: String -> Defines -> String
replace' = foldl (\s (from, to) -> replace from to s)

replaceDefs :: Defines -> Expr -> Expr
replaceDefs = traverse' . flip replace'

dropString :: String -> Parser ()
dropString = void . string

define :: Parser (String, String)
define = do
    dropString "#define"
    spaces1
    key <- atom
    spaces1
    value <- rest
    skipMany newline
    return ('#':key, value)

var :: Parser Var
var = do
    dropString "#var"
    spaces1
    v <- rest
    skipMany newline
    return (Var v)

nodeName :: Parser String
nodeName = (:) <$> upper <*> many letter <* char ':' <* skipMany newline 

selector :: Int -> Parser Expr
selector indent = do
    dropString "selector"
    spaces1
    n <- nodeName
    skipMany newline
    children <- many1 (expr (indent + 1))
    skipMany newline
    return (Selector n children)

sequence :: Int -> Parser Expr
sequence indent = do
    dropString "sequence"
    spaces1
    n <- nodeName
    skipMany newline
    children <- many1 (expr (indent + 1))
    skipMany newline
    return (Sequence n children)

cond :: Parser Expr
cond = Condition <$> (string "cond" *> spaces1 *> rest)

call :: Parser Expr
call = Call <$> (string "call" *> spaces1 *> rest)

ts :: Int -> Parser ()
ts indent = count indent (string "\t" <|> string "    ") *> pure ()

expr :: Int -> Parser Expr
expr indent = do
    try (ts indent)
    (try (selector indent)
      <|> sequence indent
      <|> try cond
      <|> call)

file :: Parser ([Var], Expr)
file = do
    defs <- many (try define)
    vars <- many (try var)
    tree <- expr 0
    return (vars, replaceDefs defs tree)

parseFile :: FilePath -> IO (Either ParseError ([Var], Expr))
parseFile fname = do
    input <- readFile fname
    return (runParser file () fname input)
