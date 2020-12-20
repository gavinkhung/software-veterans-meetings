var a = 1;
let b = 2;
const c = 3;

a = "hello";
b = "hello";

const array = [];

const fun = function(){
    console.log("hello");
    return 1;
}

const arrow = () => {
    console.log("hello from arrow function");
    return 2;
}

console.log(fun());
console.log(arrow());

console.log(array.map(item => {
    return item*2;
}));

const letters = ["a", "b"];
const [letter1, letter2] = letters;
console.log(letter1, letter2);
console.log(...letters);

const numbers = {
    num1: "this is num1",
    num2: "this is num2"
}
const {num1, num2} = numbers;
console.log(num1, num2);