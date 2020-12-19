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

export { fun };