module Main where

import System.Environment
import System.Exit
import qualified Data.Text.Lazy as L
import SeqSel.Parser
import SeqSel.Printer

tohpp :: String -> String
tohpp s = n ++ ".hpp"
  where
    n = take (length s - 3) s

main :: IO ()
main = do
    args <- getArgs
    if length args < 1 then do
        putStrLn "no input file specified"
        exitFailure
    else do
        let infile = args !! 0
        let outfile = if length args > 1 then args !! 1 else tohpp infile
        input <- parseFile infile
        case input of
            Left e -> print e
            Right (vars, expr) -> writeFile outfile (L.unpack (printTree vars expr))
